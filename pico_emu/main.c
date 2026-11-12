/*
 * daq-sim PDM emulator for the Raspberry Pi Pico 2 (RP2350).
 *
 * Replaces the 6 MEMS microphones of daq-detector: the ESP32-S3 drives the PDM clock (GPIO4) as usual,
 * the Pico answers on the three data lines (GPIO5/6/7) with the PDM bit streams of
 *   - a scenario recorded by daq-sim (tools/simulate_wav.py -> tools/pico_scenario.py), looped forever, or
 *   - a live source synthesized here (`src` mode, src_synth.h): a sinusoid moving around the array,
 *     controlled from tools/live_source.py.
 *
 *   ESP32 GPIO4 (PDM CLK)  -> Pico GP2         ESP32 GPIO5 (DATA0, M1 + M4) <- Pico GP3
 *   ESP32 GPIO6 (DATA1, M2 + M5) <- Pico GP4   ESP32 GPIO7 (DATA2, M3 + M6) <- Pico GP5      GND <-> GND
 *
 * Signal path (one PCM sample = 64 PDM clock cycles, so the sample rate follows the ESP clock):
 *   scenario (flash, int16 M1..M6) -> gain -> linear interpolation x64 -> 2nd-order sigma-delta per mic
 *   -> 64-bit masks -> packed into 24-bit words (3 lines x 8 clock halves) -> endless DMA ring -> PIO.
 *   Line j carries M(j+1) in the half after the rising edge and M(j+4) after the falling edge
 *   (body mics = SEL low = left slot, arm mics = SEL high = right slot, like DAQ_MIC_SEL_LEVEL);
 *   `swap` exchanges the halves if the ESP samples the other way round (tap test: `tone N`).
 *
 * Work split: core 0 = M1..M3 sigma-delta, packing, USB console; core 1 = M4..M6 sigma-delta.
 *
 * USB console (115200 or any baud, line based): help, play, tone N, silence, swap, gain DB, rewind,
 * quiet, info, upload BYTES (binary scenario upload used by tools/pico_scenario.py --port), and the live
 * source: src AZ EL DIST RATE, sig F0 AMP_DBFS PHASE_DEG, sweep F1 PERIOD_S, burst ON_MS OFF_MS,
 * mic N X Y Z, c MPS, fs HZ, telem 0|1 (`-` in place of a number keeps the current value).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#define PDM_RAMFUNC(name) __not_in_flash_func(name)
#include "pdm_core.h"
#include "pdm_emu.pio.h"
#include "scenario.h"
#include "src_synth.h"

#define FW_VERSION      "0.2.3"

#define PIN_CLK         2            /* from ESP32 GPIO4 */
#define PIN_DATA0       3            /* GP3, GP4, GP5 -> ESP32 GPIO5, GPIO6, GPIO7 */
#define SYS_CLOCK_KHZ   200000

#define NCH             PDM_NCH
#define OSR             PDM_OSR
#define BLOCK_FRAMES    32
#define BLOCK_WORDS     (BLOCK_FRAMES * PDM_WORDS_PER_FRAME)
#define RING_LOG2_BYTES 15           /* 32 kB ring = 8192 words = 512 samples = 10.7 ms at 48 kHz */
#define RING_WORDS      ((1u << RING_LOG2_BYTES) / 4)
#define RING_MASK       (RING_WORDS - 1)
#define RING_MARGIN     16           /* words the DMA may already hold in the PIO FIFO */

#define TONE_AMPL       3277         /* -20 dBFS */
#define TONE_TABLE      1024
#define TONE_STEP       ((uint32_t)(1000.0 / 48000.0 * 4294967296.0))   /* 1 kHz at 48 kHz */

typedef enum { MODE_SILENCE, MODE_PLAY, MODE_TONE, MODE_SRC } mode_t;

#define TELEM_PERIOD_US 100000       /* `S` lines of the live source, 10 per second */

/* ---------- shared state ---------- */
static uint32_t s_ring[RING_WORDS] __attribute__((aligned(1u << RING_LOG2_BYTES)));
static int16_t s_pcm[BLOCK_FRAMES][NCH];
static uint32_t s_masks[NCH][BLOCK_FRAMES * 2];
static sd_state_t s_sd[NCH];
static uint32_t s_lut[NCH][16];
static int16_t s_tone[TONE_TABLE];

static volatile int32_t s_gain_q12 = 4096;
static volatile uint32_t s_c1_req, s_c1_done;
static volatile uint64_t s_c1_busy_us;

static PIO s_pio = pio0;
static uint s_sm;
static int s_dma = -1;

static mode_t s_mode = MODE_SILENCE;
static int s_tone_mic;               /* 0 = all, 1..6 = only that mic */
static uint32_t s_tone_phase;
static bool s_swap;
static bool s_quiet;
static bool s_telem;
static src_t s_src;
static int s_gain_db;

static const scenario_header_t *s_scen;
static const int16_t *s_scen_data;
static uint32_t s_scen_pos, s_scen_loops;

static uint32_t s_written, s_consumed, s_last_rd, s_consumed_status;
static uint32_t s_underruns;
static uint64_t s_c0_busy_us;

/* ============================== sigma-delta (pdm_core.h) ============================== */

static void build_lut(void)
{
    pdm_build_lut(s_lut, s_swap);
}

static void __not_in_flash_func(pack_block)(void)
{
    static uint32_t *const masks[NCH] = { s_masks[0], s_masks[1], s_masks[2], s_masks[3], s_masks[4], s_masks[5] };
    uint32_t w = pdm_pack(s_ring, RING_MASK, s_written, (const uint32_t (*)[16])s_lut, masks, BLOCK_FRAMES);
    __dmb();
    s_written = w;
}

/* ============================== core 1 ============================== */

static void core1_main(void)
{
    uint32_t seen = 0;                  /* pipeline_start() clears s_c1_req before launching core 1 */
    while (true) {
        while (s_c1_req == seen) {
            __wfe();
        }
        seen = s_c1_req;
        __dmb();
        uint64_t t0 = time_us_64();
        int32_t g = s_gain_q12;
        for (int c = 3; c < NCH; c++) {
            sd_block(&s_sd[c], &s_pcm[0][c], BLOCK_FRAMES, s_masks[c], g);
        }
        s_c1_busy_us += time_us_64() - t0;
        __dmb();
        s_c1_done = seen;
        __sev();
    }
}

/* ============================== sources ============================== */

static bool scenario_valid(void)
{
    const scenario_header_t *h = (const scenario_header_t *)(XIP_BASE + SCEN_FLASH_OFFSET);
    return h->magic == SCEN_MAGIC && h->version == SCEN_VERSION && h->channels == SCEN_CHANNELS &&
           h->frames > 0 && h->data_bytes == h->frames * SCEN_CHANNELS * 2 &&
           h->data_bytes <= SCEN_MAX_BYTES - SCEN_HEADER_BYTES;
}

static void scenario_load(void)
{
    if (scenario_valid()) {
        s_scen = (const scenario_header_t *)(XIP_BASE + SCEN_FLASH_OFFSET);
        s_scen_data = (const int16_t *)(XIP_BASE + SCEN_FLASH_OFFSET + SCEN_HEADER_BYTES);
    } else {
        s_scen = NULL;
        s_scen_data = NULL;
    }
    s_scen_pos = 0;
    s_scen_loops = 0;
}

static void fill_pcm(void)
{
    if (s_mode == MODE_PLAY && s_scen) {
        for (int f = 0; f < BLOCK_FRAMES; f++) {
            memcpy(s_pcm[f], &s_scen_data[(size_t)s_scen_pos * NCH], sizeof(s_pcm[f]));
            if (++s_scen_pos >= s_scen->frames) {
                s_scen_pos = 0;
                s_scen_loops++;
            }
        }
    } else if (s_mode == MODE_TONE) {
        for (int f = 0; f < BLOCK_FRAMES; f++) {
            int16_t v = s_tone[s_tone_phase >> 22];
            s_tone_phase += TONE_STEP;
            for (int c = 0; c < NCH; c++) {
                s_pcm[f][c] = (s_tone_mic == 0 || s_tone_mic == c + 1) ? v : 0;
            }
        }
    } else if (s_mode == MODE_SRC) {
        src_block(&s_src, s_pcm, BLOCK_FRAMES);
    } else {
        memset(s_pcm, 0, sizeof(s_pcm));
    }
}

/* ============================== pipeline ============================== */

static void update_consumed(void)
{
    uint32_t rd = ((uint32_t)dma_hw->ch[s_dma].read_addr - (uint32_t)s_ring) / 4 & RING_MASK;
    s_consumed += (rd - s_last_rd) & RING_MASK;
    s_last_rd = rd;
    if ((int32_t)(s_written - s_consumed) < 0) {
        s_underruns++;                 /* the DMA ran over data not yet written: catch up */
        s_written = s_consumed;
    }
}

static uint32_t ring_free(void)
{
    int32_t used = (int32_t)(s_written - s_consumed);
    return RING_WORDS - RING_MARGIN - (uint32_t)(used < 0 ? 0 : used);
}

static void produce_block(void)
{
    uint64_t t0 = time_us_64();
    fill_pcm();
    __dmb();
    s_c1_req++;
    __sev();
    int32_t g = s_gain_q12;
    for (int c = 0; c < 3; c++) {
        sd_block(&s_sd[c], &s_pcm[0][c], BLOCK_FRAMES, s_masks[c], g);
    }
    uint64_t t1 = time_us_64();
    while (s_c1_done != s_c1_req) {
        __wfe();
    }
    __dmb();
    uint64_t t2 = time_us_64();
    pack_block();
    s_c0_busy_us += (t1 - t0) + (time_us_64() - t2);
}

static void pipeline_stop(void)
{
    pio_sm_set_enabled(s_pio, s_sm, false);
    if (s_dma >= 0) {
        dma_channel_abort(s_dma);
    }
    multicore_reset_core1();
}

static void pipeline_start(void)
{
    for (int c = 0; c < NCH; c++) {
        sd_reset(&s_sd[c]);
    }
    s_c1_req = s_c1_done = 0;
    multicore_launch_core1(core1_main);

    s_written = s_consumed = s_last_rd = s_consumed_status = 0;
    while (ring_free() >= BLOCK_WORDS) {           /* prefill: the ESP clock may already be running */
        produce_block();
    }

    pio_sm_clear_fifos(s_pio, s_sm);
    pio_sm_restart(s_pio, s_sm);
    pio_sm_exec(s_pio, s_sm, pio_encode_jmp(0));    /* program is loaded at offset 0 */

    dma_channel_config c = dma_channel_get_default_config(s_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, RING_LOG2_BYTES);
    channel_config_set_dreq(&c, pio_get_dreq(s_pio, s_sm, true));
    dma_channel_configure(s_dma, &c, &s_pio->txf[s_sm], s_ring, dma_encode_endless_transfer_count(), true);

    pio_sm_set_enabled(s_pio, s_sm, true);
}

/* ============================== upload ============================== */

static int read_byte(uint32_t timeout_us)
{
    int ch = getchar_timeout_us(timeout_us);
    return ch == PICO_ERROR_TIMEOUT ? -1 : ch;
}

static uint8_t s_chunk[FLASH_SECTOR_SIZE];

/* Protocol (tools/pico_scenario.py): "upload N\n" -> READY; then per 4 kB sector the host sends the bytes
 * and waits for "OK <offset>"; at the end DONE or ERR. The header is the first 256 bytes. */
static void do_upload(uint32_t total)
{
    if (total < SCEN_HEADER_BYTES || total > SCEN_MAX_BYTES) {
        printf("ERR size %lu (max %lu)\n", (unsigned long)total, (unsigned long)SCEN_MAX_BYTES);
        return;
    }
    pipeline_stop();
    printf("READY\n");
    stdio_flush();
    uint32_t off = 0, sum = 0;
    bool ok = true;
    while (off < total) {
        uint32_t n = total - off < FLASH_SECTOR_SIZE ? total - off : FLASH_SECTOR_SIZE;
        for (uint32_t i = 0; i < n; i++) {
            int b = read_byte(3000000);
            if (b < 0) {
                ok = false;
                break;
            }
            s_chunk[i] = (uint8_t)b;
        }
        if (!ok) {
            break;
        }
        memset(s_chunk + n, 0xFF, FLASH_SECTOR_SIZE - n);
        for (uint32_t i = (off == 0 ? SCEN_HEADER_BYTES : 0); i < n; i++) {
            sum += s_chunk[i];
        }
        uint32_t irq = save_and_disable_interrupts();
        flash_range_erase(SCEN_FLASH_OFFSET + off, FLASH_SECTOR_SIZE);
        flash_range_program(SCEN_FLASH_OFFSET + off, s_chunk, FLASH_SECTOR_SIZE);
        restore_interrupts(irq);
        off += n;
        printf("OK %lu\n", (unsigned long)off);
        stdio_flush();
    }
    scenario_load();
    if (!ok) {
        printf("ERR timeout at %lu\n", (unsigned long)off);
    } else if (!s_scen) {
        printf("ERR bad header\n");
    } else if (s_scen->checksum != sum) {
        printf("ERR checksum %08lx != %08lx\n", (unsigned long)sum, (unsigned long)s_scen->checksum);
    } else {
        printf("DONE %s %lu frames\n", s_scen->name, (unsigned long)s_scen->frames);
        s_mode = MODE_PLAY;
    }
    pipeline_start();
}

/* ============================== console ============================== */

static void print_info(void)
{
    printf("daq-sim pico_emu %s | CLK GP%d, DATA GP%d..GP%d | sys %lu MHz\n", FW_VERSION, PIN_CLK, PIN_DATA0,
           PIN_DATA0 + 2, (unsigned long)(clock_get_hz(clk_sys) / 1000000));
    if (s_scen) {
        printf("scenario: %s, %lu frames, %.2f s at %lu Hz\n", s_scen->name, (unsigned long)s_scen->frames,
               (double)s_scen->frames / s_scen->sample_rate, (unsigned long)s_scen->sample_rate);
    } else {
        printf("scenario: none (load one with tools/pico_scenario.py)\n");
    }
}

static void print_help(void)
{
    printf("commands: play | tone N (1..6, 0 = all) | silence | swap | gain DB | rewind | quiet [0|1] | info | help\n");
    printf("live source: src AZ EL DIST RATE | sig F0 AMP_DBFS PHASE_DEG | sweep F1 PERIOD_S (0 = off) | "
           "burst ON_MS OFF_MS (0 = continuous) | mic [N X Y Z] | c MPS | fs HZ | telem 0|1   (- keeps a value)\n");
}

static void print_src(void)
{
    printf("src az %.2f el %.1f dist %.3f m rate %.2f deg/s | sig %.1f Hz %.1f dBFS phase %.1f deg | "
           "sweep to %.1f Hz in %.3f s | burst %.0f / %.0f ms | c %.1f m/s fs %.0f Hz\n",
           s_src.az_deg, (double)s_src.el_deg, (double)s_src.dist_m, (double)s_src.rate_dps, (double)s_src.f0_hz,
           (double)s_src.amp_dbfs, (double)s_src.phase_deg, (double)s_src.f1_hz, (double)s_src.sweep_period_s,
           (double)s_src.burst_on_ms, (double)s_src.burst_off_ms, (double)s_src.c, (double)s_src.fs);
}

/* Up to max numbers from arg; a token that is not a number (e.g. `-`) leaves out[i] unchanged */
static int parse_floats(char *arg, float *out, int max)
{
    int n = 0;
    for (char *tok = arg ? strtok(arg, " ") : NULL; tok && n < max; tok = strtok(NULL, " "), n++) {
        char *end;
        float v = strtof(tok, &end);
        if (end != tok && *end == 0 && isfinite(v)) {
            out[n] = v;
        }
    }
    return n;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Live source commands; returns false if `cmd` is not one of them */
static bool handle_src_command(const char *cmd, char *arg)
{
    float v[4];
    if (!strcmp(cmd, "src")) {
        v[0] = (float)s_src.az_deg, v[1] = s_src.el_deg, v[2] = s_src.dist_m, v[3] = s_src.rate_dps;
        parse_floats(arg, v, 4);
        s_src.az_deg = (double)v[0] - 360.0 * floor((double)v[0] / 360.0);
        s_src.el_deg = clampf(v[1], -89.0f, 89.0f);
        s_src.dist_m = clampf(v[2], 0.2f, 100.0f);
        s_src.rate_dps = clampf(v[3], -3600.0f, 3600.0f);
        if (s_mode != MODE_SRC) {
            src_restart(&s_src);
            s_mode = MODE_SRC;
        }
    } else if (!strcmp(cmd, "sig")) {
        v[0] = s_src.f0_hz, v[1] = s_src.amp_dbfs, v[2] = s_src.phase_deg;
        parse_floats(arg, v, 3);
        s_src.f0_hz = v[0], s_src.amp_dbfs = clampf(v[1], -120.0f, 0.0f), s_src.phase_deg = v[2];
    } else if (!strcmp(cmd, "sweep")) {
        v[0] = s_src.f1_hz, v[1] = s_src.sweep_period_s;
        parse_floats(arg, v, 2);
        s_src.f1_hz = v[0], s_src.sweep_period_s = clampf(v[1], 0.0f, 3600.0f);
    } else if (!strcmp(cmd, "burst")) {
        v[0] = s_src.burst_on_ms, v[1] = s_src.burst_off_ms;
        parse_floats(arg, v, 2);
        s_src.burst_on_ms = clampf(v[0], 1.0f, 60000.0f), s_src.burst_off_ms = clampf(v[1], 0.0f, 60000.0f);
    } else if (!strcmp(cmd, "mic")) {
        v[0] = 0;
        if (parse_floats(arg, v, 4) == 4 && v[0] >= 1 && v[0] <= SRC_NCH) {
            for (int k = 0; k < 3; k++) {
                s_src.mic_m[(int)v[0] - 1][k] = clampf(v[k + 1], -400.0f, 400.0f) / 1000.0f;
            }
        } else {
            for (int m = 0; m < SRC_NCH; m++) {
                printf("mic %d %.1f %.1f %.1f\n", m + 1, (double)s_src.mic_m[m][0] * 1e3, (double)s_src.mic_m[m][1] * 1e3,
                       (double)s_src.mic_m[m][2] * 1e3);
            }
        }
    } else if (!strcmp(cmd, "c")) {
        v[0] = s_src.c;
        parse_floats(arg, v, 1);
        s_src.c = clampf(v[0], 300.0f, 380.0f);
    } else if (!strcmp(cmd, "fs")) {
        v[0] = s_src.fs;
        parse_floats(arg, v, 1);
        s_src.fs = clampf(v[0], 8000.0f, 96000.0f);
    } else if (!strcmp(cmd, "telem")) {
        s_telem = arg ? atoi(arg) != 0 : !s_telem;
        return true;
    } else {
        return false;
    }
    src_update(&s_src);
    if (!s_telem) {
        print_src();
    }
    return true;
}

/* `S az el dist rate gate`: where the source is in the signal leaving the Pico now (the ring holds the next
 * ~10 ms, so the generator is that far ahead); gate = 1 while the burst is on. */
static void telemetry(void)
{
    double ahead_s = (double)(int32_t)(s_written - s_consumed) / PDM_WORDS_PER_FRAME / s_src.fs;
    double az = s_src.az_deg - s_src.rate_dps * ahead_s;
    az -= 360.0 * floor(az / 360.0);
    printf("S %.2f %.1f %.3f %.2f %d\n", az, (double)s_src.el_deg, (double)s_src.dist_m, (double)s_src.rate_dps,
           s_src.amp > 0);
}

static void set_gain(int db)
{
    db = db < -40 ? -40 : (db > 12 ? 12 : db);
    s_gain_db = db;
    s_gain_q12 = (int32_t)lrintf(4096.0f * powf(10.0f, db / 20.0f));
}

static void handle_line(char *line)
{
    char *arg = strchr(line, ' ');
    if (arg) {
        *arg++ = 0;
    }
    if (!strcmp(line, "play")) {
        if (s_scen) {
            s_mode = MODE_PLAY;
        } else {
            printf("no scenario in flash\n");
        }
    } else if (!strcmp(line, "tone")) {
        int m = arg ? atoi(arg) : 0;
        s_tone_mic = m < 0 || m > 6 ? 0 : m;
        s_mode = MODE_TONE;
        printf("tone 1 kHz -20 dBFS on %s\n", s_tone_mic ? "one mic" : "all mics");
    } else if (!strcmp(line, "silence")) {
        s_mode = MODE_SILENCE;
    } else if (!strcmp(line, "swap")) {
        s_swap = !s_swap;
        build_lut();
        printf("swap %d: M1..M3 after the %s edge\n", s_swap, s_swap ? "falling" : "rising (default)");
    } else if (!strcmp(line, "gain")) {
        set_gain(arg ? atoi(arg) : 0);
        printf("gain %d dB\n", s_gain_db);
    } else if (!strcmp(line, "rewind")) {
        s_scen_pos = 0;
    } else if (!strcmp(line, "quiet")) {
        s_quiet = arg ? atoi(arg) != 0 : !s_quiet;   /* quiet 1 / quiet 0, no argument toggles */
    } else if (!strcmp(line, "info")) {
        print_info();
    } else if (!strcmp(line, "upload")) {
        do_upload(arg ? (uint32_t)strtoul(arg, NULL, 10) : 0);
    } else if (handle_src_command(line, arg)) {
        /* live source */
    } else if (line[0]) {
        print_help();
    }
}

static void poll_console(void)
{
    static char line[96];
    static int len;
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\r' || ch == '\n') {
            line[len] = 0;
            len = 0;
            handle_line(line);
            return;   /* handle_line may have restarted the pipeline */
        }
        if (len < (int)sizeof(line) - 1) {
            line[len++] = (char)ch;
        }
    }
}

static void status(uint32_t dt_us, uint32_t words)
{
    static const char *const names[] = { "silence", "play", "tone", "src" };
    double clk_hz = words * 4.0 * 1e6 / dt_us;   /* 4 PDM cycles per word */
    double c1 = s_c1_busy_us;
    s_c1_busy_us = 0;
    double load0 = 100.0 * s_c0_busy_us / dt_us, load1 = 100.0 * c1 / dt_us;
    s_c0_busy_us = 0;
    gpio_put(PICO_DEFAULT_LED_PIN, clk_hz > 1e5 ? !gpio_get(PICO_DEFAULT_LED_PIN) : 0);
    if (s_quiet) {
        return;
    }
    if (clk_hz < 1e5) {
        printf("clk none (waiting for the ESP32 PDM clock on GP%d) | %s | underruns %lu\n", PIN_CLK, names[s_mode],
               (unsigned long)s_underruns);
        return;
    }
    printf("clk %.3f MHz (fs %.0f Hz) | %s", clk_hz / 1e6, clk_hz / OSR, names[s_mode]);
    if (s_mode == MODE_PLAY && s_scen) {
        printf(" %s %.1f/%.1f s loop %lu", s_scen->name, (double)s_scen_pos / s_scen->sample_rate,
               (double)s_scen->frames / s_scen->sample_rate, (unsigned long)s_scen_loops);
        if (fabs(clk_hz / OSR - s_scen->sample_rate) > 0.01 * s_scen->sample_rate) {
            printf(" (scenario is %lu Hz!)", (unsigned long)s_scen->sample_rate);
        }
    } else if (s_mode == MODE_TONE) {
        printf(" M%d", s_tone_mic);
    } else if (s_mode == MODE_SRC) {
        printf(" az %.1f el %.0f d %.2f m %+.1f deg/s", s_src.az_deg, (double)s_src.el_deg, (double)s_src.dist_m,
               (double)s_src.rate_dps);
    }
    printf(" | cpu %.0f%% / %.0f%% | underruns %lu | swap %d gain %d dB\n", load0, load1,
           (unsigned long)s_underruns, s_swap, s_gain_db);
}

/* ============================== main ============================== */

int main(void)
{
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    for (int i = 0; i < TONE_TABLE; i++) {
        s_tone[i] = (int16_t)lrintf(TONE_AMPL * sinf(2.0f * (float)M_PI * i / TONE_TABLE));
    }
    build_lut();
    set_gain(0);
    src_init(&s_src, 48000.0f);
    scenario_load();
    s_mode = s_scen ? MODE_PLAY : MODE_SILENCE;

    for (int i = 0; i < 3; i++) {
        gpio_set_slew_rate(PIN_DATA0 + i, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(PIN_DATA0 + i, GPIO_DRIVE_STRENGTH_8MA);
    }
    s_sm = (uint)pio_claim_unused_sm(s_pio, true);
    uint offset = pio_add_program_at_offset(s_pio, &pdm_emu_program, 0);
    pdm_emu_program_init(s_pio, s_sm, offset, PIN_CLK, PIN_DATA0);
    s_dma = dma_claim_unused_channel(true);
    pipeline_start();

    uint64_t t_status = time_us_64(), t_telem = t_status;
    while (true) {
        update_consumed();
        if (ring_free() >= BLOCK_WORDS) {
            produce_block();
            continue;
        }
        poll_console();
        uint64_t now = time_us_64();
        if (now - t_telem >= TELEM_PERIOD_US) {
            t_telem = now;
            if (s_telem && s_mode == MODE_SRC) {
                update_consumed();
                telemetry();
            }
        }
        if (now - t_status >= 1000000) {
            update_consumed();
            status((uint32_t)(now - t_status), s_consumed - s_consumed_status);
            s_consumed_status = s_consumed;
            t_status = now;
        }
    }
}

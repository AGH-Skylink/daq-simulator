#pragma once
/*
 * Live test source of the emulator (`src` mode): a sinusoid (tone or triangular sweep, optional bursts)
 * on a circle around the array, rendered for the six microphones with the exact per-mic delay and 1/r gain.
 * Fixed-point version of daq-sim/daq_sim/live.py (the specification, same equations and constants);
 * header-only so the firmware (main.c) and the PC test (test/host_test.c) run the same code.
 *
 *   source:  P(n+1) = P(n) + f(n)/fs,  f = f0 or f0 + (f1 - f0) tri(n / (T fs));  a(n) = A * burst gate, slewed
 *   mic m:   D_m(n) = (d_m(n) - dist) / c * fs + SRC_DELAY0;   x_m(n) = dist/d_m * a(n - D_m) * sin(2 pi P(n - D_m) + phase)
 *
 * P and a are kept in a 256-sample history and read at the fractional index n - D_m (linear interpolation);
 * D_m is evaluated at the block edges and interpolated inside the block.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef PDM_RAMFUNC
#define PDM_RAMFUNC(name) name
#endif

#define SRC_NCH         6
#define SRC_RING        256          /* history in samples, power of 2 */
#define SRC_DELAY0      64.0f        /* samples, as DELAY0 in live.py; mics must stay within 0.4 m of the origin */
#define SRC_RAMP_S      0.005f       /* amplitude slew: full scale in 5 ms */
#define SRC_SINE_BITS   10
#define SRC_AMP_SHIFT   8            /* amplitude state in Q15 << 8 */

typedef struct {
    /* parameters */
    float fs, c;
    float mic_m[SRC_NCH][3];
    float el_deg, dist_m, rate_dps;
    float f0_hz, f1_hz, sweep_period_s, amp_dbfs, phase_deg, burst_on_ms, burst_off_ms;
    /* derived */
    uint32_t inc0, phase_off, burst_on, burst_period;
    uint64_t sweep_inc;              /* sweep position per sample, 2^64 = one period */
    int64_t dinc;
    int32_t amp_target, amp_step;
    /* state */
    double az_deg;
    uint32_t n, phase, burst_n;
    uint64_t sweep_u;
    int32_t amp;
    bool primed;
    int32_t delay_q16[SRC_NCH], gain_q20[SRC_NCH];
    uint32_t ph_ring[SRC_RING];
    int16_t a_ring[SRC_RING];
    int16_t sine[(1 << SRC_SINE_BITS) + 1];
} src_t;

static inline void src_update(src_t *s)
{
    float nyq = 0.45f * s->fs;
    float f0 = fminf(fmaxf(s->f0_hz, 1.0f), nyq), f1 = fminf(fmaxf(s->f1_hz, 1.0f), nyq);
    s->inc0 = (uint32_t)llround((double)f0 / s->fs * 4294967296.0);
    s->dinc = llround((double)f1 / s->fs * 4294967296.0) - (int64_t)s->inc0;
    s->sweep_inc = s->sweep_period_s > 0 ? (uint64_t)(18446744073709551616.0 / ((double)s->sweep_period_s * s->fs)) : 0;
    s->phase_off = (uint32_t)(int64_t)llround((double)s->phase_deg / 360.0 * 4294967296.0);
    float level = powf(10.0f, fminf(s->amp_dbfs, 0.0f) / 20.0f);
    s->amp_target = (int32_t)lrintf(level * 32767.0f) << SRC_AMP_SHIFT;
    s->amp_step = (int32_t)lrintf(32767.0f * (1 << SRC_AMP_SHIFT) / (SRC_RAMP_S * s->fs));
    s->burst_on = (uint32_t)lrintf(s->burst_on_ms * s->fs / 1000.0f);
    s->burst_period = s->burst_off_ms > 0 ? s->burst_on + (uint32_t)lrintf(s->burst_off_ms * s->fs / 1000.0f) : 0;
    if (s->burst_period && s->burst_n >= s->burst_period) {
        s->burst_n = 0;
    }
}

/* Defaults = LiveParams() in live.py and the geometry of daq-detector/main/config.h */
static inline void src_init(src_t *s, float fs)
{
    static const float mic_mm[SRC_NCH][3] = {
        { 0.0f, 34.6f, 0.0f },      { -30.0f, -17.3f, 0.0f },   { 30.0f, -17.3f, 0.0f },
        { -303.1f, 175.0f, -161.0f }, { 0.0f, -350.0f, -161.0f }, { 303.1f, 175.0f, -161.0f },
    };
    memset(s, 0, sizeof(*s));
    s->fs = fs;
    s->c = 343.0f;
    for (int m = 0; m < SRC_NCH; m++) {
        for (int k = 0; k < 3; k++) {
            s->mic_m[m][k] = mic_mm[m][k] / 1000.0f;
        }
    }
    s->dist_m = 2.0f;
    s->rate_dps = 30.0f;
    s->f0_hz = 500.0f;
    s->f1_hz = 3000.0f;
    s->sweep_period_s = 0.02f;
    s->amp_dbfs = -12.0f;
    s->burst_on_ms = 800.0f;
    s->burst_off_ms = 500.0f;
    for (int i = 0; i <= 1 << SRC_SINE_BITS; i++) {
        s->sine[i] = (int16_t)lrint(32767.0 * sin(2.0 * M_PI * i / (1 << SRC_SINE_BITS)));
    }
    src_update(s);
}

/* Restart the signal from sample 0 (sweep, bursts, phase, history); the position is kept */
static inline void src_restart(src_t *s)
{
    s->n = s->phase = s->burst_n = 0;
    s->sweep_u = 0;
    s->amp = 0;
    s->primed = false;
    memset(s->ph_ring, 0, sizeof(s->ph_ring));
    memset(s->a_ring, 0, sizeof(s->a_ring));
}

/* Delay (Q16 samples) and gain (Q20) of every mic for the source at azimuth az_deg */
static inline void src_geometry(const src_t *s, double az_deg, int32_t delay_q16[SRC_NCH], int32_t gain_q20[SRC_NCH])
{
    float az = (float)(az_deg * (M_PI / 180.0)), el = s->el_deg * (float)(M_PI / 180.0);
    float ce = cosf(el);
    float p[3] = { s->dist_m * ce * cosf(az), s->dist_m * ce * sinf(az), s->dist_m * sinf(el) };
    for (int m = 0; m < SRC_NCH; m++) {
        float dx = p[0] - s->mic_m[m][0], dy = p[1] - s->mic_m[m][1], dz = p[2] - s->mic_m[m][2];
        float d = sqrtf(dx * dx + dy * dy + dz * dz);
        d = d < 0.05f ? 0.05f : d;
        float delay = (d - s->dist_m) / s->c * s->fs + SRC_DELAY0;
        delay = delay < 1.0f ? 1.0f : (delay > SRC_RING - 2 ? SRC_RING - 2 : delay);
        delay_q16[m] = (int32_t)lrintf(delay * 65536.0f);
        float g = s->dist_m / d;
        gain_q20[m] = (int32_t)lrintf((g > 7.99f ? 7.99f : g) * 1048576.0f);
    }
}

/* Next n samples (n <= 64) of the six mics, interleaved M1..M6 */
static void PDM_RAMFUNC(src_block)(src_t *s, int16_t (*pcm)[SRC_NCH], int n)
{
    int32_t d0[SRC_NCH], g0[SRC_NCH];
    if (!s->primed) {
        src_geometry(s, s->az_deg, s->delay_q16, s->gain_q20);
        s->primed = true;
    }
    memcpy(d0, s->delay_q16, sizeof(d0));     /* continue from where the last block ended */
    memcpy(g0, s->gain_q20, sizeof(g0));
    double az_end = s->az_deg + (double)s->rate_dps * n / s->fs;
    src_geometry(s, az_end, s->delay_q16, s->gain_q20);

    /* source: phase and amplitude history */
    uint32_t n0 = s->n;
    for (int i = 0; i < n; i++) {
        uint32_t inc = s->inc0;
        if (s->sweep_inc) {
            uint32_t u = (uint32_t)(s->sweep_u >> 32);
            uint32_t tri = u < 0x80000000u ? u : 0u - u;   /* 0 .. 2^31 = f0 .. f1 */
            inc += (uint32_t)(int32_t)((s->dinc * (int64_t)tri) >> 31);
            s->sweep_u += s->sweep_inc;
        }
        int32_t target = s->amp_target;
        if (s->burst_period) {
            if (s->burst_n >= s->burst_on) {
                target = 0;
            }
            if (++s->burst_n >= s->burst_period) {
                s->burst_n = 0;
            }
        }
        int32_t da = target - s->amp;
        s->amp += da > s->amp_step ? s->amp_step : (da < -s->amp_step ? -s->amp_step : da);
        uint32_t k = (n0 + (uint32_t)i) & (SRC_RING - 1);
        s->ph_ring[k] = s->phase;
        s->a_ring[k] = (int16_t)(s->amp >> SRC_AMP_SHIFT);
        s->phase += inc;
    }

    /* microphones: history read at n - D_m */
    for (int m = 0; m < SRC_NCH; m++) {
        int32_t d = d0[m], dd = (s->delay_q16[m] - d0[m]) / n, g = g0[m], dg = (s->gain_q20[m] - g0[m]) / n;
        for (int i = 0; i < n; i++) {
            uint32_t pos = ((n0 + (uint32_t)i) << 16) - (uint32_t)d;
            uint32_t k = (pos >> 16) & (SRC_RING - 1), k1 = (k + 1) & (SRC_RING - 1);
            int32_t fr = (int32_t)(pos & 0xFFFF);
            uint32_t ph = s->ph_ring[k] + (uint32_t)(int32_t)(((int64_t)(int32_t)(s->ph_ring[k1] - s->ph_ring[k]) * fr) >> 16);
            int32_t a = s->a_ring[k] + (((s->a_ring[k1] - s->a_ring[k]) * (fr >> 1)) >> 15);
            ph += s->phase_off;
            uint32_t ti = ph >> (32 - SRC_SINE_BITS);
            int32_t tf = (int32_t)((ph >> (32 - SRC_SINE_BITS - 15)) & 0x7FFF);
            int32_t sn = s->sine[ti] + (((s->sine[ti + 1] - s->sine[ti]) * tf) >> 15);
            int32_t v = (int32_t)(((int64_t)((sn * a) >> 15) * g) >> 20);
            pcm[i][m] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
            d += dd;
            g += dg;
        }
    }
    s->n = n0 + (uint32_t)n;
    s->az_deg = az_end;
    if (s->az_deg >= 360.0 || s->az_deg < 0.0) {
        s->az_deg -= 360.0 * floor(s->az_deg / 360.0);
    }
}

/*
 * PC test of the emulator core: raw 6-channel int16 PCM (stdin) -> sd_block + pdm_pack (same code as the
 * firmware) -> model of the receiver: the 3 data lines are split into the 6 mics by clock half (as the
 * ESP32 PDM RX with DAQ_MIC_SEL_LEVEL {0,0,0,1,1,1}) and decoded with a sinc^4 decimator (R = 64)
 * -> raw 6-channel int16 PCM (stdout). tools/pico_check.py compares the result with the input.
 *
 *   cc -O2 -I.. host_test.c -o host_test && ./host_test [swap] < in.raw > out.raw
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pdm_core.h"

#define BLOCK 32

int main(int argc, char **argv)
{
    bool swap = argc > 1 && !strcmp(argv[1], "swap");
    static uint32_t lut[PDM_NCH][16];
    pdm_build_lut(lut, swap);
    sd_state_t sd[PDM_NCH];
    for (int c = 0; c < PDM_NCH; c++) {
        sd_reset(&sd[c]);
    }
    static uint32_t masks_mem[PDM_NCH][BLOCK * 2];
    uint32_t *masks[PDM_NCH];
    for (int c = 0; c < PDM_NCH; c++) {
        masks[c] = masks_mem[c];
    }
    static uint32_t words[BLOCK * PDM_WORDS_PER_FRAME];
    int16_t pcm[BLOCK][PDM_NCH];

    /* receiver: sinc^4 decimator per mic */
    int64_t integ[PDM_NCH][4] = { 0 }, comb[PDM_NCH][4] = { 0 };
    int phase = 0;
    size_t got;
    while ((got = fread(pcm, PDM_NCH * 2, BLOCK, stdin)) > 0) {
        if (got < BLOCK) {
            memset(pcm[got], 0, (BLOCK - got) * sizeof(pcm[0]));
        }
        for (int c = 0; c < PDM_NCH; c++) {
            sd_block(&sd[c], &pcm[0][c], BLOCK, masks[c], 4096);
        }
        pdm_pack(words, 0xFFFFFFFFu, 0, (const uint32_t (*)[16])lut, masks, BLOCK);

        for (int w = 0; w < BLOCK * PDM_WORDS_PER_FRAME; w++) {
            for (int cyc = 0; cyc < 4; cyc++) {
                /* the receiver takes mic j+1 (left slot) from the half after the rising edge and mic j+4
                 * (right slot) from the half after the falling edge, as measured on the ESP32-S3 */
                int bits[PDM_NCH];
                for (int line = 0; line < 3; line++) {
                    bits[line] = words[w] >> (3 * (2 * cyc) + line) & 1;
                    bits[line + 3] = words[w] >> (3 * (2 * cyc + 1) + line) & 1;
                }
                for (int c = 0; c < PDM_NCH; c++) {
                    int64_t v = bits[c] ? 1 : -1;
                    for (int k = 0; k < 4; k++) {
                        integ[c][k] += v;
                        v = integ[c][k];
                    }
                }
                if (++phase == PDM_OSR) {
                    phase = 0;
                    int16_t out[PDM_NCH];
                    for (int c = 0; c < PDM_NCH; c++) {
                        int64_t v = integ[c][3];
                        for (int k = 0; k < 4; k++) {
                            int64_t t = v;
                            v -= comb[c][k];
                            comb[c][k] = t;
                        }
                        double y = (double)v / (1 << 24) * 32768.0;   /* gain R^4 = 2^24 */
                        out[c] = (int16_t)(y > 32767 ? 32767 : (y < -32768 ? -32768 : y));
                    }
                    fwrite(out, sizeof(out), 1, stdout);
                }
            }
        }
    }
    return 0;
}

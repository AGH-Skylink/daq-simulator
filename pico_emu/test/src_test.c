/*
 * PC test of the live source (src_synth.h, same code as the firmware): writes raw 6-channel int16 PCM.
 * Compare with daq_sim/live.py (tools/live_source.py --wav gives the same signal from the Python model):
 *
 *   cc -O2 -I.. src_test.c -lm -o src_test
 *   ./src_test AZ EL DIST RATE F0 AMP_DBFS PHASE_DEG F1 SWEEP_S ON_MS OFF_MS SECONDS > out.raw
 */
#include <stdio.h>
#include <stdlib.h>
#include "src_synth.h"

#define BLOCK 32

int main(int argc, char **argv)
{
    if (argc != 13) {
        fprintf(stderr, "usage: src_test AZ EL DIST RATE F0 AMP_DBFS PHASE_DEG F1 SWEEP_S ON_MS OFF_MS SECONDS\n");
        return 1;
    }
    static src_t s;
    src_init(&s, 48000.0f);
    s.az_deg = atof(argv[1]);
    s.el_deg = (float)atof(argv[2]);
    s.dist_m = (float)atof(argv[3]);
    s.rate_dps = (float)atof(argv[4]);
    s.f0_hz = (float)atof(argv[5]);
    s.amp_dbfs = (float)atof(argv[6]);
    s.phase_deg = (float)atof(argv[7]);
    s.f1_hz = (float)atof(argv[8]);
    s.sweep_period_s = (float)atof(argv[9]);
    s.burst_on_ms = (float)atof(argv[10]);
    s.burst_off_ms = (float)atof(argv[11]);
    src_update(&s);
    src_restart(&s);
    int16_t pcm[BLOCK][SRC_NCH];
    for (long left = (long)(atof(argv[12]) * 48000.0); left > 0; left -= BLOCK) {
        src_block(&s, pcm, BLOCK);
        fwrite(pcm, sizeof(pcm[0]), left < BLOCK ? (size_t)left : BLOCK, stdout);
    }
    return 0;
}

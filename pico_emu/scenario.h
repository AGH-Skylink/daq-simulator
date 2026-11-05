#pragma once
/*
 * Scenario stored in the Pico 2 flash: a 6-channel 16-bit PCM recording (M1..M6 interleaved, as written
 * by tools/simulate_wav.py and daq-detector). Written by tools/pico_scenario.py, either over USB
 * (`upload` command, see main.c) or as a UF2 file at SCEN_FLASH_OFFSET.
 *
 * Layout at SCEN_FLASH_OFFSET:  256-byte header (scenario_header_t, rest zero), then frames * 6 * int16.
 */
#include <stdint.h>

#define SCEN_FLASH_OFFSET   (512u * 1024u)                 /* firmware must stay below 512 kB */
#define SCEN_FLASH_SIZE     (4u * 1024u * 1024u)           /* Pico 2: 4 MB flash */
#define SCEN_MAX_BYTES      (SCEN_FLASH_SIZE - SCEN_FLASH_OFFSET)
#define SCEN_HEADER_BYTES   256u
#define SCEN_MAGIC          0x43535144u                    /* "DQSC" little-endian */
#define SCEN_VERSION        1u
#define SCEN_CHANNELS       6u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t channels;
    uint32_t sample_rate;      /* informative: the emulator outputs one sample per 64 ESP clock cycles */
    uint32_t frames;
    uint32_t data_bytes;       /* frames * channels * 2 */
    uint32_t checksum;         /* sum of all data bytes as uint32, modulo 2^32 */
    char     name[32];         /* source file name, NUL terminated */
} scenario_header_t;

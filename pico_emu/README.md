# pico_emu — 6-microphone PDM emulator (Raspberry Pi Pico 2)

Hardware-in-the-loop test for `daq-detector`: the Pico 2 takes the place of the 6 MEMS microphones and plays a
scenario from `daq-sim` (e.g. a speaker at 60° / 20° / 2 m made by `tools/simulate_wav.py`) as real PDM bit
streams. The ESP32-S3 runs its normal firmware (acquisition, DSP, localization, tracker, BLE), so the whole
live path is tested against a known ground truth, PDM front end included — unlike the SD replay mode,
which starts after the PDM decoder.

## Wiring

The ESP32-S3 stays the PDM master: it drives the clock, the Pico answers on the data lines.

| ESP32-S3 (daq-detector) | | Pico 2 | Signal |
|---|---|---|---|
| GPIO4 (`DAQ_PIN_PDM_CLK`) | → | GP2 (pin 4) | PDM clock 3.072 MHz |
| GPIO5 (`DAQ_PIN_PDM_DIN0`) | ← | GP3 (pin 5) | DATA0 = M1 (left slot) + M4 (right slot) |
| GPIO6 (`DAQ_PIN_PDM_DIN1`) | ← | GP4 (pin 6) | DATA1 = M2 + M5 |
| GPIO7 (`DAQ_PIN_PDM_DIN2`) | ← | GP5 (pin 7) | DATA2 = M3 + M6 |
| GND | — | GND (pin 3) | common ground |

- Disconnect the microphones (at least their DAT wires) from the ESP32, they would drive the same lines.
- The SEL GPIOs of the ESP32 are not used.
- Do not connect 3V3 / 5V between the boards: each one is powered from its own USB (or the UPS HAT).
- Keep the wires short (< 15 cm); a 33–100 Ω series resistor in the CLK wire helps against ringing,
  a double-counted clock edge would exchange left and right.

## How it works

```
flash: scenario (int16 M1..M6) ─▶ gain ─▶ x64 linear interpolation ─▶ 2nd-order sigma-delta per mic
       ─▶ 64-bit masks ─▶ 24-bit words (3 lines × 8 clock halves) ─▶ endless DMA ring (32 kB) ─▶ PIO
PIO:   wait CLK high → out 3 bits → wait CLK low → out 3 bits      (edge to output ≈ 20 ns at 200 MHz)
```

- One PCM sample = 64 PDM clock cycles, so the sample rate follows the ESP clock (48 kHz by default; a
  32 kHz firmware works with a 32 kHz scenario).
- Line j carries M(j+1) in the half after the rising edge and M(j+4) after the falling edge (body mics =
  SEL low = left slot, arm mics = SEL high, as `DAQ_MIC_SEL_LEVEL`). If the ESP takes the other half,
  M1..M3 and M4..M6 come out exchanged: `swap` on the console fixes it.
- Core 0: M1..M3 modulators, packing, USB console; core 1: M4..M6 modulators (~50–60 % each at 200 MHz,
  shown in the status line).
- The scenario (up to ~6 s at 48 kHz) is stored in the Pico flash from 512 kB on and looped.

Checked on the PC with the same C code (`test/host_test.c`, receiver model = split by clock half + sinc⁴
decimator): the received signal matches the scenario with the same delay on all channels (spread
0.0001 samples), GCC-PHAT time differences differ by 0.004 µs (median), and `localize_wav.py` gives
60.0° / 20.1° / 2.01 m on the scenario at 60° / 20° / 2 m.

## Build

Pico SDK 2.x and `arm-none-eabi-gcc`:

```
cd pico_emu
mkdir build && cd build
cmake -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk ..
ninja                                   # -> daq_pico_emu.uf2
```

## Use

1. **Firmware** (once): hold BOOTSEL on the Pico, plug it in, copy `daq_pico_emu.uf2` to the `RP2350` drive.
2. **Scenario:**
   ```
   python tools/simulate_wav.py data/sim.wav --source 60 20 2.0 --seconds 5
   python tools/pico_scenario.py data/sim.wav --port COM9        # the Pico's USB serial port
   ```
   (alternative: `--uf2 data/sim.uf2` and copy it in BOOTSEL mode). It stays in flash after power-off.
3. **Console** (any terminal on the Pico's port): one status line per second, e.g.
   `clk 3.072 MHz (fs 48000 Hz) | play sim.wav 1.3/5.0 s loop 2 | cpu 55% / 51% | underruns 0 | swap 0 gain 0 dB`.
   `clk none` = no clock from the ESP32 (check GPIO4 → GP2 and that capture is running).

   | Command | Effect |
   |---|---|
   | `play` | play the scenario (default when one is loaded) |
   | `tone N` | 1 kHz, −20 dBFS on mic N only (1..6), `tone 0` on all; channel mapping test |
   | `silence` | digital silence |
   | `swap` | exchange the clock halves (M1..M3 ↔ M4..M6) |
   | `gain DB` | −40..+12 dB |
   | `rewind` | scenario from the start |
   | `quiet [0\|1]` | status lines off / on |
   | `info` | firmware, pins, scenario |
4. **Mapping test:** `tone 1` … `tone 6` and watch the level of M1..M6 on the ESP status line (UART console)
   or in `usb_record.py`: only the selected mic must rise by ~40 dB. M1 ↔ M4 exchanged → `swap`.
5. **Check the received audio:**
   ```
   python tools/usb_record.py --port COM7 --out data/usb_test.wav --seconds 10
   python tools/pico_check.py data/sim.wav data/usb_test.wav
   ```
   Expected: every channel matches itself, delay spread < 1 µs, GCC-PHAT median difference < 1 µs, `result: OK`.
6. **Check the device result:** record on the ESP SD card (button) and compare its CSV with the Python reference on
   the same audio, then with the ground truth of the scenario:
   ```
   python tools/localize_wav.py data/REC00001.WAV --compare data/REC00001.CSV
   ```
   The speaker of `sim.wav` is at 60° / 20° / 2 m.

## Files

| File | Purpose |
|---|---|
| `main.c` | pipeline (DMA ring, two cores), scenario / tone sources, USB console, upload |
| `pdm_core.h` | sigma-delta modulator and bit packing (shared with the PC test) |
| `pdm_emu.pio` | PIO program: 3 bits on every clock edge |
| `scenario.h` | flash layout of the scenario (header + PCM) |
| `test/host_test.c` | PC model: same code + receiver (split by half, sinc⁴ decimator) |
| `../tools/pico_scenario.py` | WAV → Pico (USB upload or UF2) |
| `../tools/pico_check.py` | received recording vs scenario: mapping, delays, GCC-PHAT |

PC test:
```
cc -O2 -Ipico_emu pico_emu/test/host_test.c -o host_test
python -c "from daq_sim.recording import load_recording as l; open('in.raw','wb').write(l('data/sim.wav')[0].tobytes())"
./host_test < in.raw > out.raw      # then wrap out.raw as a 6-channel 48 kHz WAV and run tools/pico_check.py
```

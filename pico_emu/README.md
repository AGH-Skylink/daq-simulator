# pico_emu — 6-microphone PDM emulator (Raspberry Pi Pico 2)

Hardware-in-the-loop test for `daq-detector`: the Pico 2 takes the place of the 6 MEMS microphones and plays a
scenario from `daq-sim` (e.g. a speaker at 60° / 20° / 2 m made by `tools/simulate_wav.py`) as real PDM bit
streams. The ESP32-S3 runs its normal firmware (acquisition, DSP, localization, tracker, BLE), so the whole
live path is tested against a known ground truth, PDM front end included — unlike the SD replay mode,
which starts after the PDM decoder.

Step-by-step guide with the wiring drawing and the live source window: [`../docs/hardware_in_the_loop.md`](../docs/hardware_in_the_loop.md).

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
PIO:   wait CLK high → out 3 bits (M4..M6) → wait CLK low → out 3 bits (M1..M3)      (edge to output ≈ 20 ns at 200 MHz)
```

- One PCM sample = 64 PDM clock cycles, so the sample rate follows the ESP clock (48 kHz by default; a
  32 kHz firmware works with a 32 kHz scenario).
- Clock: the Pico has no sample clock of its own. The PIO outputs one bit per line after every edge of the
  ESP clock, so the bit rate is the ESP clock exactly (no drift); the receiver samples it on the next edge.
- Line j carries M(j+1) in the half after the falling edge and M(j+4) after the rising edge, like the real
  microphones (ST AN5027: select low = data valid while CLK is low; ESP-IDF: left slot = select pulled down;
  body mics = SEL low, arm mics = SEL high, as `DAQ_MIC_SEL_LEVEL`). If the channels still come out
  exchanged (M1..M3 ↔ M4..M6), `swap` on the console exchanges the halves.
- Core 0: source, M1..M3 modulators, packing, USB console; core 1: M4..M6 modulators (load of both cores
  is shown in the status line; not yet measured on hardware).
- The scenario (up to ~6 s at 48 kHz) is stored in the Pico flash from 512 kB on and looped.

Checked on the PC with the same C code (`test/host_test.c`, receiver model = split by clock half + sinc⁴
decimator): the received signal matches the scenario with the same delay on all channels (spread
0.0001 samples), GCC-PHAT time differences differ by 0.004 µs (median), and `localize_wav.py` gives
60.0° / 20.1° / 2.01 m on the scenario at 60° / 20° / 2 m.

## Live source (`src` mode)

Besides playing a stored scenario the Pico can synthesize a test source itself: a sinusoid (tone or triangular
sweep, optional bursts) at a given azimuth / elevation / distance, rotating around the array at a given speed.
Every mic gets the source with its exact delay and 1/r gain (`src_synth.h`, fixed-point version of
`daq_sim/live.py`), so only parameters travel over USB and the signal is tied to the ESP clock.

`tools/live_source.py` is the front end: a window with the array, the source and its parameters
(the Pico is found automatically; the **Wiring...** button shows which Pico pin goes to which ESP32 GPIO):

```
python tools/live_source.py --az 0 --dist 2 --rate 30        # the Pico is found by itself; --port COM9 to choose
```

| Command | Effect (`-` in place of a number keeps the current value) |
|---|---|
| `src AZ EL DIST RATE` | position (deg, deg, m) and rotation speed (deg/s); switches to the live source |
| `sig F0 AMP_DBFS PHASE_DEG` | sinusoid: frequency, level at the array origin, phase |
| `sweep F1 PERIOD_S` | triangular sweep F0 → F1 → F0 (`PERIOD_S` 0 = pure tone) |
| `burst ON_MS OFF_MS` | bursts (`OFF_MS` 0 = continuous) |
| `mic N X Y Z`, `c MPS`, `fs HZ` | mic position in mm (as on the ESP console), speed of sound, sample rate |
| `telem 0\|1` | 10 lines per second `S az el dist rate gate`: the source as it is in the signal leaving the Pico |

Defaults: 500 → 3000 Hz sweep in 20 ms, bursts 800 / 500 ms, −12 dBFS, geometry of `config.h`. The device
localizes from time differences over 300–3500 Hz and its VAD floor follows a stationary signal, so a
continuous pure tone is detected for a few seconds only and hardly ever localized (reference algorithm: 22
of 220 voice frames); the sweep in bursts is localized in every frame.

Checked on the PC: `test/src_test.c` (the firmware code) matches `daq_sim/live.py` within a few LSB, and
through the PDM model (`test/host_test.c`) the reference algorithm A follows a source rotating at
30 / −60 / 120 deg/s with a median azimuth error of 0.08° / 0.15° / 0.28°.

## Build

Pico SDK 2.x and `arm-none-eabi-gcc`:

```
cd pico_emu
mkdir build && cd build
cmake -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk ..
ninja                                   # -> daq_pico_emu.uf2
```

## Use

1. **Firmware** (once): hold BOOTSEL on the Pico, plug it in, copy `daq_pico_emu.uf2` (built, or `prebuilt/daq_pico_emu.uf2`)
   to the `RP2350` drive.
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
| `src_synth.h` | live source: moving sinusoid for the six mics (shared with the PC test) |
| `scenario.h` | flash layout of the scenario (header + PCM) |
| `test/host_test.c` | PC model: same code + receiver (split by half, sinc⁴ decimator) |
| `test/src_test.c` | PC run of the live source (raw PCM, compare with `tools/live_source.py --wav`) |
| `../tools/live_source.py` | window: array, source, parameters, wiring; drives the `src` mode |
| `../tools/pico_scenario.py` | WAV → Pico (USB upload or UF2) |
| `../tools/pico_check.py` | received recording vs scenario: mapping, delays, GCC-PHAT |

PC test:
```
cc -O2 -Ipico_emu pico_emu/test/host_test.c -o host_test
python -c "from daq_sim.recording import load_recording as l; open('in.raw','wb').write(l('data/sim.wav')[0].tobytes())"
./host_test < in.raw > out.raw      # then wrap out.raw as a 6-channel 48 kHz WAV and run tools/pico_check.py
```

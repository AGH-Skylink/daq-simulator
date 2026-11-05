"""Load a 6-channel WAV into the Pico 2 PDM emulator (pico_emu) as the scenario it plays to daq-detector.

    python tools/pico_scenario.py data/sim.wav --port COM9            # upload over USB (Pico running pico_emu)
    python tools/pico_scenario.py data/sim.wav --uf2 data/sim.uf2     # or: UF2 to drag onto the Pico in BOOTSEL mode

The WAV must be 16-bit, 6 channels M1..M6 (tools/simulate_wav.py, usb_record.py or the SD card), at the
sample rate the ESP32 runs (DAQ_SAMPLE_RATE_HZ, default 48000). Up to ~6 s at 48 kHz fit in the flash;
--seconds cuts a longer file. The scenario is looped: start and end it with silence (simulate_wav.py
starts with 0.5 s of silence; --fade adds a short fade so the loop point does not click).
"""
import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from daq_sim.recording import load_recording  # noqa: E402

# must match pico_emu/scenario.h
FLASH_OFFSET = 512 * 1024
FLASH_SIZE = 4 * 1024 * 1024
MAX_BYTES = FLASH_SIZE - FLASH_OFFSET
HEADER_BYTES = 256
MAGIC = 0x43535144
VERSION = 1
XIP_BASE = 0x10000000
UF2_FAMILY_ABSOLUTE = 0xE48BFF57   # RP2350: write at the absolute flash address


def build_image(samples, fs, name):
    data = samples.astype("<i2").tobytes()
    checksum = sum(data) & 0xFFFFFFFF
    header = struct.pack("<IHHIIII32s", MAGIC, VERSION, 6, fs, samples.shape[0], len(data), checksum,
                         name.encode()[:31])
    return header.ljust(HEADER_BYTES, b"\0") + data


def write_uf2(image, path):
    blocks = [image[i:i + 256] for i in range(0, len(image), 256)]
    with open(path, "wb") as f:
        for n, chunk in enumerate(blocks):
            chunk = chunk.ljust(256, b"\xff")
            hdr = struct.pack("<IIIIIIII", 0x0A324655, 0x9E5D5157, 0x2000, XIP_BASE + FLASH_OFFSET + 256 * n,
                              256, n, len(blocks), UF2_FAMILY_ABSOLUTE)
            f.write(hdr + chunk + b"\0" * (476 - 256) + struct.pack("<I", 0x0AB16F30))
    print(f"{path}: {len(blocks)} blocks; hold BOOTSEL, plug the Pico in and copy the file to the RP2350 drive")


def read_line(ser, timeout):
    end = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < end:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            line = buf.decode(errors="replace").strip()
            buf = b""
            if line:
                return line
            continue
        buf += b
    return None


def wait_for(ser, prefixes, timeout):
    """Skips status lines until one starting with any of `prefixes` arrives."""
    while True:
        line = read_line(ser, timeout)
        if line is None:
            return None
        if line.startswith(prefixes):
            return line


def upload(image, port):
    import serial
    ser = serial.Serial(port, 115200, timeout=0.1)
    ser.reset_input_buffer()
    ser.write(b"\nquiet 1\n")            # stop the status lines during the upload
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(f"upload {len(image)}\n".encode())
    line = wait_for(ser, ("READY", "ERR"), 5)
    if line != "READY":
        sys.exit(f"Pico did not accept the upload: {line} (is pico_emu running on {port}?)")
    t0 = time.monotonic()
    last_pct = -1
    print(f"uploading {len(image) / 1e6:.2f} MB (the flash writes ~60-100 kB/s, about {len(image) / 70e3:.0f} s)")
    for off in range(0, len(image), 4096):
        ser.write(image[off:off + 4096])
        line = wait_for(ser, ("OK", "ERR"), 10)
        if not line or not line.startswith("OK"):
            sys.exit(f"upload failed at {off}: {line}")
        pct = 100 * min(off + 4096, len(image)) // len(image)
        if pct != last_pct:
            last_pct = pct
            print(f"\r  {pct:3d}%  {(off + 4096) / 1e3 / (time.monotonic() - t0):5.0f} kB/s", end="", flush=True)
    line = wait_for(ser, ("DONE", "ERR"), 10)
    print()
    ser.write(b"quiet 0\n")              # status lines back on
    ser.close()
    if not line or not line.startswith("DONE"):
        sys.exit(f"upload failed: {line}")
    print(f"Pico: {line}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav")
    ap.add_argument("--port", help="USB serial port of the Pico running pico_emu (Windows COMx, Linux /dev/ttyACM0)")
    ap.add_argument("--uf2", help="write a UF2 file instead of uploading")
    ap.add_argument("--seconds", type=float, help="use only the first SECONDS of the file")
    ap.add_argument("--fade", type=float, default=0.02, help="fade-in/out at the loop point in s (0 = off)")
    args = ap.parse_args()
    if not args.port and not args.uf2:
        sys.exit("give --port or --uf2")

    samples, fs = load_recording(args.wav)
    if samples.shape[1] != 6:
        sys.exit(f"{args.wav}: {samples.shape[1]} channels, the emulator needs 6 (M1..M6)")
    max_frames = (MAX_BYTES - HEADER_BYTES) // 12
    n = samples.shape[0]
    if args.seconds:
        n = min(n, int(args.seconds * fs))
    if n > max_frames:
        print(f"note: only the first {max_frames / fs:.2f} s fit in the flash ({n / fs:.2f} s given)")
        n = max_frames
    x = samples[:n].astype(np.float64)
    nf = int(args.fade * fs)
    if nf > 0 and 2 * nf < n:
        ramp = np.linspace(0.0, 1.0, nf)[:, None]
        x[:nf] *= ramp
        x[-nf:] *= ramp[::-1]
    x = np.round(x).astype(np.int16)
    peak = np.abs(x).max() / 32768
    if peak > 0.75:
        print(f"warning: peak {20 * np.log10(peak):.1f} dBFS; the emulator clips at -2.5 dBFS (use `gain` or a quieter file)")
    image = build_image(x, fs, Path(args.wav).name)
    print(f"{args.wav}: {n} frames, {n / fs:.2f} s at {fs} Hz, {len(image) / 1e6:.2f} MB")

    if args.uf2:
        write_uf2(image, args.uf2)
    if args.port:
        upload(image, args.port)


if __name__ == "__main__":
    main()

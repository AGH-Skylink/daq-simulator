"""Check what the device received from the Pico 2 PDM emulator (pico_emu) against the scenario that was played.

    python tools/pico_check.py data/sim.wav data/usb_test.wav          # ESP32 recording (usb_record.py or SD card)
    python tools/pico_check.py data/sim.wav data/host.wav               # PC model (pico_emu/test, see README)

The scenario loops on the Pico, so the recording may start anywhere in it: the offset is found by
correlation (circular over the scenario). Then, per channel: which scenario channel it matches best
(catches swapped halves -> `swap` on the Pico console), gain, SNR and the residual delay (all channels
must have the same delay, otherwise the time differences are biased), and finally the GCC-PHAT time
differences of daq_sim.tdoa on both signals (target: median |difference| below 1 us).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from daq_sim.geometry import PAIRS, load_geometry  # noqa: E402
from daq_sim.recording import load_recording  # noqa: E402
from daq_sim.tdoa import TdoaEstimator  # noqa: E402


def find_offset(ref, rec, fs):
    """Offset o such that rec[n] ~ ref[(n + o) % len(ref)], from M1..M6 summed (integer samples)."""
    seg = rec[: min(len(rec), len(ref), int(3 * fs))].astype(np.float64).sum(axis=1)
    r = ref.astype(np.float64).sum(axis=1)
    n = len(r)
    tiled = np.concatenate([r, r[: len(seg)]])
    size = 1 << int(np.ceil(np.log2(len(tiled) + len(seg))))
    c = np.fft.irfft(np.fft.rfft(tiled, size) * np.conj(np.fft.rfft(seg, size)), size)[:n]
    return int(np.argmax(c))


def frac_lag(a, b):
    """Delay of a relative to b in samples (parabolic refinement of the cross-correlation peak)."""
    size = 1 << int(np.ceil(np.log2(2 * len(a))))
    c = np.fft.irfft(np.fft.rfft(a, size) * np.conj(np.fft.rfft(b, size)), size)
    c = np.concatenate([c[-50:], c[:51]])          # lags -50 .. 50
    m = int(np.argmax(c))
    if 0 < m < len(c) - 1:
        ym, y0, yp = c[m - 1], c[m], c[m + 1]
        den = ym - 2 * y0 + yp
        d = 0.5 * (ym - yp) / den if den else 0.0
    else:
        d = 0.0
    return m - 50 + d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", help="WAV that was loaded into the Pico (tools/pico_scenario.py input)")
    ap.add_argument("recording", help="6-channel WAV recorded by the device (or by the PC model)")
    ap.add_argument("--config", help="daq-detector/main/config.h for the geometry")
    ap.add_argument("--skip", type=float, default=0.2, help="seconds skipped at the start of the recording")
    args = ap.parse_args()

    ref, fs_ref = load_recording(args.scenario)
    rec, fs = load_recording(args.recording)
    if ref.shape[1] != 6 or rec.shape[1] != 6:
        sys.exit("both files must have 6 channels")
    if fs != fs_ref:
        print(f"warning: sample rates differ ({fs_ref} vs {fs} Hz)")
    rec = rec[int(args.skip * fs):]
    if len(rec) < 2048 * 4:
        sys.exit("recording too short")

    off = find_offset(ref, rec, fs)
    idx = (np.arange(len(rec)) + off) % len(ref)
    al = ref[idx]
    print(f"offset in scenario: {off} samples ({off / fs:.3f} s), compared {len(rec) / fs:.1f} s")

    x = rec.astype(np.float64)
    y = al.astype(np.float64)
    # channel matching: normalised correlation at the common delay
    print("\nchannel  best match  corr   gain     SNR    delay (samples)")
    lags = []
    swapped = False
    for c in range(6):
        corr = [np.dot(x[:, c], y[:, k]) / (np.linalg.norm(x[:, c]) * np.linalg.norm(y[:, k]) + 1e-12) for k in range(6)]
        best = int(np.argmax(corr))
        lag = frac_lag(x[:, c], y[:, c])
        lags.append(lag)
        shift = int(round(lag))
        yy = np.roll(y[:, c], shift)
        g = np.dot(x[:, c], yy) / (np.dot(yy, yy) + 1e-12)
        e = x[:, c] - g * yy
        snr = 10 * np.log10(np.dot(g * yy, g * yy) / (np.dot(e, e) + 1e-12))
        flag = "" if best == c else "  <-- matches M%d" % (best + 1)
        swapped |= best != c
        print(f"  M{c + 1}        M{best + 1}      {corr[best]:.3f}  {g:6.3f}  {snr:5.1f} dB  {lag:+.3f}{flag}")
    spread = max(lags) - min(lags)
    print(f"delay spread between channels: {spread:.4f} samples = {spread / fs * 1e6:.2f} us")

    g = load_geometry(args.config)
    shift = int(round(np.median(lags)))
    rec_al = rec[shift:] if shift >= 0 else rec
    al2 = al[: len(rec_al)] if shift >= 0 else al[-shift:]
    n = min(len(rec_al), len(al2))
    res_rec = TdoaEstimator(g).run(rec_al[:n])
    res_ref = TdoaEstimator(g).run(al2[:n])
    d = [np.abs(a.tau_s - b.tau_s) * 1e6 for a, b in zip(res_rec, res_ref) if a.voice and b.voice]
    print(f"\nGCC-PHAT on {len(d)} voice frames (of {len(res_ref)}):")
    if d:
        d = np.array(d)
        print(f"  |tau difference| median {np.median(d):.3f} us, p90 {np.percentile(d, 90):.3f} us, max {d.max():.2f} us")
        worst = np.argsort(np.median(d, axis=0))[::-1][:3]
        print("  worst pairs: " + ", ".join(f"{PAIRS[p][0] + 1}{PAIRS[p][1] + 1} {np.median(d[:, p]):.2f} us" for p in worst))
    ok = not swapped and spread / fs * 1e6 < 1.0 and len(d) and np.median(d) < 1.0
    print("\nresult:", "OK" if ok else "CHECK")
    if swapped:
        print("  channels do not match the scenario: if M1..M3 <-> M4..M6 are exchanged, type `swap` on the Pico console")


if __name__ == "__main__":
    main()

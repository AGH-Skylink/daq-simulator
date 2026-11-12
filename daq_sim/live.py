"""Live test source: a sinusoid on a circle around the array, as seen by the six microphones.

This is the specification of the `src` mode of the Pico 2 emulator (pico_emu/src_synth.h runs the same
equations in fixed point) and the model behind tools/live_source.py.

Source signal, sample n (fs = sample rate of the device):
- frequency f(n) = f0, or a triangular sweep f0 -> f1 -> f0 with period T: f = f0 + (f1 - f0) tri(n / (T fs)),
- phase P(n + 1) = P(n) + f(n) / fs (cycles), so frequency changes never break the phase,
- amplitude a(n): A (linear, from dBFS) times the burst gate (on for `burst_on_ms`, off for `burst_off_ms`,
  off = 0 means continuous), slewed by at most full scale per RAMP_S so nothing clicks.
  The device VAD follows a stationary signal with its noise floor (+0.05 dB per frame), so a continuous tone
  stops counting as voice after a few seconds: bursts keep it detectable.

Geometry: the source is at (azimuth, elevation, distance), azimuth advancing by `rate_dps` degrees per second.
Mic m at distance d_m(n) gets the source delayed by the path difference to the array origin and scaled by 1/r:

    D_m(n) = (d_m(n) - dist) / c * fs + DELAY0          (samples; DELAY0 keeps it positive)
    x_m(n) = dist / d_m(n) * a(n - D_m) * sin(2 pi P(n - D_m) + phase)

with P and a read at the fractional index n - D_m by linear interpolation. Only the delay differences
between the mics matter to the device; a moving source gets its Doppler shift from D_m changing in time.
"""
from dataclasses import dataclass, replace

import numpy as np

from .geometry import from_spherical

DELAY0 = 64.0        # samples; must exceed max |mic position| / c * fs (54 samples for the default array)
RAMP_S = 0.005       # amplitude slew: full scale in 5 ms


@dataclass(frozen=True)
class LiveParams:
    az_deg: float = 0.0
    el_deg: float = 0.0
    dist_m: float = 2.0
    rate_dps: float = 30.0          # azimuth speed, degrees per second (negative = clockwise seen from above)
    freq_hz: float = 500.0
    amp_dbfs: float = -12.0         # level at the array origin
    phase_deg: float = 0.0
    sweep_to_hz: float = 3000.0
    sweep_period_s: float = 0.02    # 0 = no sweep (pure tone at freq_hz)
    burst_on_ms: float = 800.0
    burst_off_ms: float = 500.0     # 0 = continuous

    def with_(self, **kw):
        return replace(self, **kw)


def source_position(p, t=0.0):
    """Position (m) at time t after the parameters were set."""
    return from_spherical(p.az_deg + p.rate_dps * t, p.el_deg, p.dist_m)


def mic_delays(geometry, pos_m, dist_m):
    """Per-mic (delay relative to the array origin in seconds, gain relative to the origin)."""
    d = np.linalg.norm(geometry.mics_m - np.asarray(pos_m, dtype=float), axis=1)
    return (d - dist_m) / geometry.c, dist_m / d


def source_signal(p, fs, n):
    """Source phase (cycles) and amplitude (linear, 1 = full scale) for samples 0..n-1."""
    k = np.arange(n)
    if p.sweep_period_s > 0:
        pos = (k / (p.sweep_period_s * fs)) % 1.0
        tri = np.where(pos < 0.5, 2 * pos, 2 - 2 * pos)
        f = p.freq_hz + (p.sweep_to_hz - p.freq_hz) * tri
    else:
        f = np.full(n, float(p.freq_hz))
    phase = np.concatenate([[0.0], np.cumsum(f[:-1] / fs)])

    level = 10 ** (p.amp_dbfs / 20)
    if p.burst_off_ms > 0:
        on = int(round(p.burst_on_ms * fs / 1000))
        period = on + int(round(p.burst_off_ms * fs / 1000))
        target = np.where(k % period < on, level, 0.0)
    else:
        target = np.full(n, level)
    step = 1.0 / (RAMP_S * fs)
    amp = np.empty(n)
    a = 0.0
    for i in range(n):                       # slew limiter (a(n) uses the gate of sample n)
        a += min(max(target[i] - a, -step), step)
        amp[i] = a
    return phase, amp


def render_live(geometry, p, seconds):
    """6-channel int16 (n, 6) of the live source, the signal the emulator feeds to the device."""
    fs = geometry.fs
    n = int(seconds * fs)
    phase, amp = source_signal(p, fs, n)
    t = np.arange(n) / fs
    az = np.radians(p.az_deg + p.rate_dps * t)
    el = np.radians(p.el_deg)
    pos = p.dist_m * np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.full(n, np.sin(el))], axis=1)
    out = np.zeros((n, 6))
    idx = np.arange(n, dtype=float)
    for m in range(6):
        d = np.linalg.norm(pos - geometry.mics_m[m], axis=1)
        at = idx - ((d - p.dist_m) / geometry.c * fs + DELAY0)
        ph = np.interp(at, idx, phase)       # before sample 0: amplitude 0, so the phase does not matter
        a = np.interp(at, idx, amp, left=0.0)
        out[:, m] = p.dist_m / d * a * np.sin(2 * np.pi * ph + np.radians(p.phase_deg))
    return np.clip(np.round(out * 32767), -32768, 32767).astype(np.int16)

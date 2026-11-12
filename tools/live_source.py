"""Live test source for daq-detector: a window with the mic array and a sound source moving around it.

    python tools/live_source.py --az 0 --dist 2 --rate 30                    # finds the Pico 2 by itself
    python tools/live_source.py --az 0 --dist 2 --rate 30 --port COM9        # or: the Pico on a given port
    python tools/live_source.py --az 0 --dist 2 --rate 30 --no-pico          # window only (simulation)
    python tools/live_source.py --az 60 --dist 2 --rate 0 --wav data/live.wav --seconds 10   # same signal to a WAV

The source is a sinusoid (tone or sweep, optional bursts) at (azimuth, elevation, distance), its azimuth
advancing by `rate` degrees per second. With the Pico 2 running pico_emu connected, it synthesizes exactly this
source for the six microphones (`src` mode, model: daq_sim/live.py) and feeds it to the ESP32 as PDM; the
window then shows the azimuth reported by the Pico, i.e. what is in the signal right now. Every change in
the window (sliders, fields) is sent to the Pico immediately. The Pico is found by its USB id; if none is
connected the script says so and the window runs the same model on the PC clock. --esp-port adds what
the device reports on its UART console (status line, once per second).

The device localizes from time differences over 300-3500 Hz and its VAD needs pauses, so the defaults are
a 500 -> 3000 Hz sweep (20 ms) in bursts (0.8 s on / 0.5 s off); a pure continuous tone is not localizable.
"""
import argparse
import math
import queue
import re
import sys
import threading
import time
import wave
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from daq_sim.geometry import from_spherical, load_geometry  # noqa: E402
from daq_sim.live import LiveParams, mic_delays, render_live  # noqa: E402

SCENE_PX = 600          # scene canvas (top view), square
ZOOM_PX = 300           # array close-up
ZOOM_HALF_M = 0.45      # half width of the close-up in metres
WAVE_H = 200            # mic signal panel height
TICK_MS = 40            # redraw period
SEND_MS = 60            # minimum time between parameter updates sent to the Pico
TRAIL_S = 1.5           # length of the source trail

BG, GRID, INK, MUTED = "#fbfbf9", "#dcdcd6", "#1f2328", "#6b7075"
C_SRC, C_ESP, C_BODY, C_ARM = "#d9480f", "#1864ab", "#2b8a3e", "#5f3dc4"
MIC_COLORS = ["#2b8a3e", "#37b24d", "#74b816", "#5f3dc4", "#7950f2", "#ae3ec9"]

# name: (label, unit, min, max, slider resolution, format)
SLIDERS = {
    "az": ("Azimuth", "deg", 0.0, 360.0, 0.5, "{:.1f}"),
    "rate": ("Rotation speed", "deg/s", -180.0, 180.0, 1.0, "{:.1f}"),
    "dist": ("Distance", "m", 0.5, 10.0, 0.05, "{:.2f}"),
    "el": ("Elevation", "deg", -30.0, 80.0, 1.0, "{:.1f}"),
    "freq": ("Frequency", "Hz", 100.0, 8000.0, 10.0, "{:.0f}"),
    "amp": ("Amplitude", "dBFS", -60.0, -3.0, 0.5, "{:.1f}"),
    "phase": ("Phase", "deg", 0.0, 360.0, 1.0, "{:.0f}"),
}


# Wiring of the emulator (pico_emu/main.c: PIN_CLK = GP2, PIN_DATA0 = GP3..GP5) and the board pinouts
PICO_LEFT = ["GP0", "GP1", "GND", "GP2", "GP3", "GP4", "GP5", "GND", "GP6", "GP7",
             "GP8", "GP9", "GND", "GP10", "GP11", "GP12", "GP13", "GND", "GP14", "GP15"]             # pins 1..20
PICO_RIGHT = ["VBUS", "VSYS", "GND", "3V3_EN", "3V3", "ADC_VREF", "GP28", "GND", "GP27", "GP26",
              "RUN", "GP22", "GND", "GP21", "GP20", "GP19", "GP18", "GND", "GP17", "GP16"]           # pins 40..21
ESP_J1 = ["3V3", "3V3", "RST", "4", "5", "6", "7", "15", "16", "17", "18", "8", "3", "46", "9", "10",
          "11", "12", "13", "14", "5V", "G"]                                                        # pins 1..22
ESP_J3 = ["G", "TX", "RX", "1", "2", "42", "41", "40", "39", "38", "37", "36", "35", "0", "45", "48",
          "47", "21", "20", "19", "G", "G"]
# (signal, Pico pin number, config.h macro of the ESP32 GPIO, default GPIO, wire colour, direction)
WIRES = [
    ("GND", 3, None, None, "#343a40", "common ground"),
    ("PDM CLK", 4, "DAQ_PIN_PDM_CLK", 4, "#e67700", "ESP32 -> Pico"),
    ("DATA0 (M1 + M4)", 5, "DAQ_PIN_PDM_DIN0", 5, "#2b8a3e", "Pico -> ESP32"),
    ("DATA1 (M2 + M5)", 6, "DAQ_PIN_PDM_DIN1", 6, "#1864ab", "Pico -> ESP32"),
    ("DATA2 (M3 + M6)", 7, "DAQ_PIN_PDM_DIN2", 7, "#862e9c", "Pico -> ESP32"),
]


def load_pdm_pins(config_h):
    """ESP32 GPIO numbers of the PDM clock and data lines from config.h (defaults: 4, 5, 6, 7)."""
    pins = {macro: gpio for _, _, macro, gpio, _, _ in WIRES if macro}
    if config_h:
        text = Path(config_h).read_text(encoding="utf-8")
        for macro in pins:
            m = re.search(rf"#define\s+{macro}\s+GPIO_NUM_(\d+)", text)
            if m:
                pins[macro] = int(m.group(1))
    return pins


def open_wiring(root, tk, pins):
    """Window with both boards seen from above (USB up) and the five wires between them."""
    win = tk.Toplevel(root)
    win.title("Wiring: ESP32-S3-DevKitC-1 <-> Pico 2")
    win.configure(bg=BG)
    w, h = 1060, 690
    c = tk.Canvas(win, width=w, height=h, bg="white", highlightthickness=0)
    c.pack(padx=10, pady=10)
    tiny, small, bold = ("TkDefaultFont", 7), ("TkDefaultFont", 8), ("TkDefaultFont", 9, "bold")
    pitch = 23

    def pad(x, y, colour):
        c.create_rectangle(x - 7, y - 7, x + 7, y + 7, fill=colour or "#e9c46a", outline="#8a6d1b")

    # ---- ESP32-S3-DevKitC-1, turned by 180 deg (USB ports up, antenna down) so that header J1 faces the Pico:
    #      J1 on the right edge with pin 1 at the bottom, J3 on the left edge with pin 1 at the bottom
    ex0, ex1, ey0 = 60, 240, 50
    ey_pin1 = ey0 + 56 + 21 * pitch
    ey1 = ey_pin1 + 62
    esp_y = lambda k: ey_pin1 - (k - 1) * pitch                # noqa: E731  (k = header pin 1..22)
    c.create_text((ex0 + ex1) / 2, 16, text="ESP32-S3-DevKitC-1", fill=INK, font=bold)
    c.create_text((ex0 + ex1) / 2, 32, text="top view, USB ports up, antenna down", fill=MUTED, font=small)
    c.create_rectangle(ex0, ey0, ex1, ey1, fill="#25262b", outline="#000000", width=2)
    for i, name in enumerate(("USB", "UART")):                 # the two ports; which one is left depends on the revision
        x = ex0 + 52 + i * 76
        c.create_rectangle(x - 22, ey0 - 12, x + 22, ey0 + 20, fill="#ced4da", outline="#868e96")
    c.create_text((ex0 + ex1) / 2, ey0 + 32, text="2 x USB (labelled USB and UART)", fill="#adb5bd", font=tiny)
    c.create_rectangle(ex0 + 42, ey1 - 150, ex1 - 42, ey1 - 34, fill="#adb5bd", outline="#868e96")     # module shield
    c.create_text((ex0 + ex1) / 2, ey1 - 96, text="ESP32-S3\nWROOM-1", fill=INK, font=small, justify="center")
    c.create_rectangle(ex0 + 42, ey1 - 34, ex1 - 42, ey1 + 14, fill="#343a40", outline="#868e96")       # antenna
    c.create_text((ex0 + ex1) / 2, ey1 - 10, text="antenna", fill="#ced4da", font=tiny)
    c.create_text(ex0 - 14, ey1 + 4, text="J3", fill=MUTED, font=bold)
    c.create_text(ex1 + 14, ey1 + 4, text="J1", fill=INK, font=bold)

    esp_colour = {}                                            # header label -> wire colour
    for _, _, macro, _, colour, _ in WIRES:
        if macro:
            esp_colour[str(pins[macro])] = colour
    gnd_pin = len(ESP_J1)                                      # J1 pin 22 = G, next to the USB ports
    for k in range(1, 23):
        y = esp_y(k)
        for header, x, side in ((ESP_J1, ex1, 1), (ESP_J3, ex0, 0)):
            label = header[k - 1]
            colour = esp_colour.get(label)
            if header is ESP_J1 and k == gnd_pin:
                colour = WIRES[0][4]
            pad(x, y, colour)
            c.create_text(x + (-12 if side else 12), y, text=label, anchor="e" if side else "w",
                          fill="white", font=bold if colour else small)
            if side:
                c.create_text(x + 12, y, text=str(k), anchor="w", fill=INK if colour else MUTED,
                              font=bold if colour else tiny)

    # ---- Pico 2, USB connector up: pins 1..20 on the left edge, 40..21 on the right edge
    bx0, bx1, by0 = 560, 730, 50
    pico_y = lambda i: by0 + 56 + i * pitch                    # noqa: E731  (i = 0..19 from the USB end)
    by1 = pico_y(19) + 34
    c.create_text((bx0 + bx1) / 2, 16, text="Raspberry Pi Pico 2", fill=INK, font=bold)
    c.create_text((bx0 + bx1) / 2, 32, text="top view, USB up", fill=MUTED, font=small)
    c.create_rectangle(bx0, by0, bx1, by1, fill="#2f9e44", outline="#237032", width=2)
    c.create_rectangle((bx0 + bx1) / 2 - 26, by0 - 12, (bx0 + bx1) / 2 + 26, by0 + 20, fill="#ced4da", outline="#868e96")
    c.create_text((bx0 + bx1) / 2, by0 + 4, text="USB", fill=INK, font=small)
    c.create_rectangle((bx0 + bx1) / 2 - 24, by0 + 296, (bx0 + bx1) / 2 + 24, by0 + 344, fill="#212529", outline="")
    c.create_text((bx0 + bx1) / 2, by0 + 320, text="RP2350", fill="white", font=small)
    c.create_oval(bx0 + 52, by0 + 30, bx0 + 68, by0 + 46, fill="#f1f3f5", outline="#868e96")
    c.create_text(bx0 + 96, by0 + 38, text="BOOTSEL", fill="white", font=tiny)
    pico_colour = {pin: colour for _, pin, _, _, colour, _ in WIRES}
    for i in range(20):
        y = pico_y(i)
        for side, x, label, number in ((0, bx0, PICO_LEFT[i], i + 1), (1, bx1, PICO_RIGHT[i], 40 - i)):
            colour = pico_colour.get(number) if side == 0 else None
            pad(x, y, colour)
            c.create_text(x + (12 if side == 0 else -12), y, text=label, anchor="w" if side == 0 else "e",
                          fill="white", font=bold if colour else small)
            c.create_text(x + (-12 if side == 0 else 12), y, text=str(number), anchor="e" if side == 0 else "w",
                          fill=INK if colour else MUTED, font=bold if colour else small)

    # ---- wires: out of the ESP32 header, up a lane of its own, into the Pico pin.
    #      Both headers count away from their pin 1 in opposite directions, so the four signal wires cross once each.
    rows = []
    lane = 0
    for name, pin, macro, _, colour, direction in WIRES:
        if macro:
            label = str(pins[macro])
            if label in ESP_J1:
                k, x_from, where = ESP_J1.index(label) + 1, ex1 + 26, f"J1 pin {ESP_J1.index(label) + 1}"
            elif label in ESP_J3:
                k, x_from, where = ESP_J3.index(label) + 1, ex0 + 8, f"J3 pin {ESP_J3.index(label) + 1}"
            else:
                rows.append((colour, f"GPIO{label}: not on a header of the DevKitC-1", name, pin, direction))
                continue
            esp_text = f"GPIO{label}  (label \u00ab{label}\u00bb, {where})"
        else:
            k, x_from, esp_text = gnd_pin, ex1 + 26, f"GND  (label \u00abG\u00bb, J1 pin {gnd_pin})"
        rows.append((colour, esp_text, name, pin, direction))
        y_from, y_to = esp_y(k), pico_y(pin - 1)
        x_lane = ex1 + 70 if not macro else bx0 - 60 - 34 * lane
        lane += 1 if macro else 0
        arrow = {"ESP32 -> Pico": "last", "Pico -> ESP32": "first"}.get(direction, "none")
        c.create_line(x_from, y_from, x_lane, y_from, x_lane, y_to, bx0 - 30, y_to, fill=colour, width=4,
                      arrow=arrow, arrowshape=(12, 14, 6), joinstyle="round")
        c.create_text(x_from + 24, y_from - 9, text=name, anchor="w", fill=colour, font=small)

    # ---- connection list and notes
    tx = bx1 + 50
    c.create_text(tx, 16, text="Connections", anchor="w", fill=INK, font=bold)
    y = 50
    for colour, esp_text, name, pin, direction in rows:
        c.create_rectangle(tx, y - 6, tx + 12, y + 6, fill=colour, outline="")
        c.create_text(tx + 20, y, anchor="w", fill=INK, font=bold, text=name)
        c.create_text(tx + 20, y + 16, anchor="w", fill=INK, font=small, text=f"ESP32  {esp_text}")
        c.create_text(tx + 20, y + 30, anchor="w", fill=INK, font=small,
                      text=f"Pico     pin {pin}  ({PICO_LEFT[pin - 1]})")
        c.create_text(tx + 20, y + 44, anchor="w", fill=MUTED, font=small, text=direction)
        y += 70
    notes = ["Before powering up:",
             "\u2022 unplug the microphones' DAT wires from the ESP32 (they would drive the same lines),",
             "\u2022 leave the SEL GPIOs of the ESP32 unconnected,",
             "\u2022 do not connect 3V3 or 5V between the boards: each one has its own USB cable,",
             "\u2022 keep the five wires short (under 15 cm).",
             "",
             "Pico pins are counted from the USB end, left column 1..20.",
             "On the ESP32 go by the labels printed next to the pins: J1 is the header "
             "that starts with 3V3, 3V3, RST at the antenna end."]
    c.create_text(tx, y + 6, anchor="nw", fill=INK, font=("TkDefaultFont", 9), text="\n".join(notes), width=w - tx - 12)
    return win


class SerialLines(threading.Thread):
    """Reads text lines from a serial port into a queue; write() sends one command line."""

    def __init__(self, port):
        super().__init__(daemon=True)
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.lines = queue.Queue()
        self.alive = True
        self.start()

    def run(self):
        buf = b""
        while self.alive:
            try:
                data = self.ser.read(256)
            except Exception:                     # port unplugged
                self.lines.put(None)
                return
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.lines.put(line.decode(errors="replace").strip())

    def write(self, text):
        try:
            self.ser.write((text + "\n").encode())
        except Exception:
            self.lines.put(None)

    def close(self):
        self.alive = False
        try:
            self.ser.close()
        except Exception:
            pass


def open_port(port, option):
    """SerialLines on `port`, or exit with the list of ports that do exist."""
    import serial
    from serial.tools import list_ports
    try:
        return SerialLines(port)
    except serial.SerialException as e:
        ports = [f"  {p.device:8s} {p.description}" + ("   <- Pico" if p.vid == 0x2E8A else "") for p in list_ports.comports()]
        sys.exit(f"{option} {port}: cannot open the port ({e}).\nPorts on this computer:\n"
                 + ("\n".join(ports) if ports else "  none (is the board plugged in?)")
                 + "\nWithout --port the window runs as a simulation only.")


def find_pico():
    """Serial port of a Raspberry Pi Pico (USB vendor id 2E8A), or None. In BOOTSEL mode the Pico has no port."""
    from serial.tools import list_ports
    ports = [p.device for p in list_ports.comports() if p.vid == 0x2E8A]
    if len(ports) > 1:
        print(f"several Picos found ({', '.join(ports)}), using {ports[0]}; choose another one with --port")
    return ports[0] if ports else None


def triggered_waves(p, tau_s, gain, n=260):
    """Mic signals for the scope panel, triggered at the start of a sweep (or at phase 0 of a tone):
    returns (t_ms, x[6, n]) with x_m(t) = gain_m sin(2 pi P(t - tau_m) + phase)."""
    if p.sweep_period_s > 0:
        window = min(p.sweep_period_s, 0.008)
    else:
        window = min(4.0 / max(p.freq_hz, 1.0), 0.02)
    pre = 0.0015
    t = np.linspace(-pre, window + pre, n + 120)
    if p.sweep_period_s > 0:
        pos = (np.clip(t, 0, None) / p.sweep_period_s) % 1.0
        f = p.freq_hz + (p.sweep_to_hz - p.freq_hz) * np.where(pos < 0.5, 2 * pos, 2 - 2 * pos)
    else:
        f = np.full(t.shape, float(p.freq_hz))
    phase = np.cumsum(f) * (t[1] - t[0])
    phase -= np.interp(0.0, t, phase)
    tt = np.linspace(0.0, window, n)
    x = np.array([gain[m] * np.sin(2 * np.pi * np.interp(tt - tau_s[m], t, phase) + math.radians(p.phase_deg))
                  for m in range(6)])
    return tt * 1e3, x


class App:
    def __init__(self, root, tk, ttk, geometry, params, pico, esp, pdm_pins):
        self.root, self.tk, self.ttk = root, tk, ttk
        self.pdm_pins = pdm_pins
        self.g = geometry
        self.p = params
        self.pico, self.esp = pico, esp
        self.az = params.az_deg % 360.0
        self.sound = True
        self.t_last = time.monotonic()
        self.t_start = self.t_last
        self.trail = []                       # (time, az, el, dist)
        self.dirty = set()
        self.t_sent = 0.0
        self.t_telem = 0.0                    # last `S` line from the Pico
        self.t_az_sent = 0.0                  # last azimuth change sent to the Pico
        self.pico_gate = True                 # burst gate reported by the Pico
        self.pico_status = "connecting..." if pico else ""
        self.esp_src = None                   # (time, az, el, dist)
        self.esp_spk = ""
        self.updating = False

        root.title("daq-sim live source")
        root.configure(bg=BG)
        left = tk.Frame(root, bg=BG)
        left.grid(row=0, column=0, padx=(10, 6), pady=10, sticky="n")
        right = tk.Frame(root, bg=BG)
        right.grid(row=0, column=1, padx=(6, 10), pady=10, sticky="n")

        self.scene = tk.Canvas(left, width=SCENE_PX, height=SCENE_PX, bg="white", highlightthickness=1,
                               highlightbackground=GRID)
        self.scene.pack()
        self.readout = tk.Label(left, bg=BG, fg=INK, font=("TkFixedFont", 10), anchor="w", justify="left",
                                wraplength=SCENE_PX)
        self.readout.pack(fill="x", pady=(8, 0))
        self.status = tk.Label(left, bg=BG, fg=MUTED, font=("TkDefaultFont", 9), anchor="w", justify="left",
                               wraplength=SCENE_PX)
        self.status.pack(fill="x")

        top = tk.Frame(right, bg=BG)
        top.pack(fill="x")
        self.zoom = tk.Canvas(top, width=ZOOM_PX, height=ZOOM_PX, bg="white", highlightthickness=1,
                              highlightbackground=GRID)
        self.zoom.grid(row=0, column=0)
        self.waves = tk.Canvas(top, width=ZOOM_PX, height=WAVE_H, bg="white", highlightthickness=1,
                               highlightbackground=GRID)
        self.waves.grid(row=1, column=0, pady=(8, 0))
        self.build_controls(right)
        self.draw_zoom_static()

        if pico:
            self.send_all()
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    # ------------------------------------------------------------------ controls

    def build_controls(self, parent):
        tk, ttk = self.tk, self.ttk
        box = tk.Frame(parent, bg=BG)
        box.pack(fill="x", pady=(10, 0))
        self.vars, self.texts, self.entries, self.shown = {}, {}, {}, {}
        start = {"az": self.az, "rate": self.p.rate_dps, "dist": self.p.dist_m, "el": self.p.el_deg,
                 "freq": self.p.freq_hz, "amp": self.p.amp_dbfs, "phase": self.p.phase_deg}
        for row, (name, (label, unit, lo, hi, res, fmt)) in enumerate(SLIDERS.items()):
            tk.Label(box, text=label, bg=BG, fg=INK, anchor="w", width=13).grid(row=row, column=0, sticky="w")
            var = tk.DoubleVar(value=start[name])
            text = tk.StringVar(value=fmt.format(start[name]))
            scale = tk.Scale(box, variable=var, from_=lo, to=hi, resolution=res, orient="horizontal", length=150,
                             showvalue=False, bg=BG, highlightthickness=0, troughcolor=GRID,
                             command=lambda _v, n=name: self.on_slider(n))
            scale.grid(row=row, column=1, padx=4)
            entry = ttk.Entry(box, textvariable=text, width=7, justify="right")
            entry.grid(row=row, column=2)
            entry.bind("<Return>", lambda _e, n=name: self.on_entry(n))
            entry.bind("<FocusOut>", lambda _e, n=name: self.on_entry(n))
            tk.Label(box, text=unit, bg=BG, fg=MUTED, anchor="w", width=5).grid(row=row, column=3, sticky="w")
            self.vars[name], self.texts[name], self.entries[name] = var, text, entry
            self.shown[name] = text.get()

        extra = tk.Frame(parent, bg=BG)
        extra.pack(fill="x", pady=(6, 0))
        self.sweep_on = tk.BooleanVar(value=self.p.sweep_period_s > 0)
        self.sweep_to = tk.StringVar(value=f"{self.p.sweep_to_hz:.0f}")
        self.sweep_t = tk.StringVar(value=f"{(self.p.sweep_period_s or 0.02) * 1e3:.0f}")
        self.burst_on = tk.BooleanVar(value=self.p.burst_off_ms > 0)
        self.burst_a = tk.StringVar(value=f"{self.p.burst_on_ms:.0f}")
        self.burst_b = tk.StringVar(value=f"{(self.p.burst_off_ms or 500):.0f}")
        rows = [(self.sweep_on, "Sweep to", self.sweep_to, "Hz in", self.sweep_t, "ms"),
                (self.burst_on, "Bursts on", self.burst_a, "ms, off", self.burst_b, "ms")]
        for row, (flag, label, a, mid, b, unit) in enumerate(rows):
            ttk.Checkbutton(extra, text=label, variable=flag, command=self.on_extra).grid(row=row, column=0, sticky="w")
            for col, (var, after) in enumerate(((a, mid), (b, unit))):
                e = ttk.Entry(extra, textvariable=var, width=6, justify="right")
                e.grid(row=row, column=1 + 2 * col, padx=(6, 3))
                e.bind("<Return>", lambda _e: self.on_extra())
                e.bind("<FocusOut>", lambda _e: self.on_extra())
                tk.Label(extra, text=after, bg=BG, fg=MUTED).grid(row=row, column=2 + 2 * col, sticky="w")

        buttons = tk.Frame(parent, bg=BG)
        buttons.pack(fill="x", pady=(8, 0))
        self.sound_btn = ttk.Button(buttons, text="Sound: on", width=12, command=self.toggle_sound)
        self.sound_btn.pack(side="left")
        ttk.Button(buttons, text="Stop rotation", command=lambda: self.set_value("rate", 0.0)).pack(side="left", padx=6)
        ttk.Button(buttons, text="Wiring...", command=lambda: open_wiring(self.root, tk, self.pdm_pins)).pack(side="left")

    def on_slider(self, name):
        if self.updating:
            return
        self.show(name, self.vars[name].get())
        self.apply(name, self.vars[name].get())

    def show(self, name, v):
        self.shown[name] = SLIDERS[name][5].format(v)
        self.texts[name].set(self.shown[name])

    def on_entry(self, name):
        _, _, lo, hi, _, fmt = SLIDERS[name]
        if self.texts[name].get() == self.shown[name]:       # not edited (e.g. focus left the field)
            return
        try:
            v = float(self.texts[name].get().replace(",", "."))
        except ValueError:
            v = self.vars[name].get()
        if name == "az":
            v %= 360.0
        elif name == "rate":
            v = min(max(v, -3600.0), 3600.0)          # faster than the slider range is allowed from the field
        else:
            v = min(max(v, lo), hi)
        self.set_value(name, v)

    def value(self, name):
        return {"az": self.az, "rate": self.p.rate_dps, "dist": self.p.dist_m, "el": self.p.el_deg,
                "freq": self.p.freq_hz, "amp": self.p.amp_dbfs, "phase": self.p.phase_deg}[name]

    def set_value(self, name, v):
        self.updating = True
        self.vars[name].set(v)
        self.updating = False
        self.show(name, v)
        self.apply(name, v)

    def apply(self, name, v):
        if name == "az":
            self.az = v % 360.0
            self.trail.clear()
        else:
            field = {"rate": "rate_dps", "dist": "dist_m", "el": "el_deg", "freq": "freq_hz", "amp": "amp_dbfs",
                     "phase": "phase_deg"}[name]
            self.p = self.p.with_(**{field: v})
        self.dirty.add(name)

    def on_extra(self):
        def num(var, lo, hi, default):
            try:
                v = float(var.get().replace(",", "."))
            except ValueError:
                v = default
            v = min(max(v, lo), hi)
            var.set(f"{v:g}")
            return v
        f1 = num(self.sweep_to, 100.0, 8000.0, self.p.sweep_to_hz)
        t_ms = num(self.sweep_t, 2.0, 60000.0, 20.0)
        on = num(self.burst_a, 10.0, 60000.0, 800.0)
        off = num(self.burst_b, 10.0, 60000.0, 500.0)
        new = self.p.with_(sweep_to_hz=f1, sweep_period_s=t_ms / 1e3 if self.sweep_on.get() else 0.0,
                           burst_on_ms=on, burst_off_ms=off if self.burst_on.get() else 0.0)
        if new != self.p:
            self.p = new
            self.dirty.add("extra")

    def toggle_sound(self):
        self.sound = not self.sound
        self.sound_btn.configure(text="Sound: on" if self.sound else "Sound: off")
        if self.pico:
            if self.sound:
                for cmd in self.sig_commands():
                    self.pico.write(cmd)
                self.pico.write(self.src_command(with_az=True))
                self.t_az_sent = time.monotonic()
                self.dirty.clear()
            else:
                self.pico.write("silence")

    # ------------------------------------------------------------------ Pico / ESP

    def src_command(self, with_az):
        az = f"{self.az:.3f}" if with_az else "-"
        return f"src {az} {self.p.el_deg:.2f} {self.p.dist_m:.3f} {self.p.rate_dps:.3f}"

    def sig_commands(self):
        p = self.p
        return [f"sig {p.freq_hz:.2f} {p.amp_dbfs:.2f} {p.phase_deg:.2f}",
                f"sweep {p.sweep_to_hz:.2f} {p.sweep_period_s:.5f}",
                f"burst {p.burst_on_ms:.1f} {p.burst_off_ms:.1f}"]

    def send_all(self):
        """Everything the Pico needs: telemetry on, geometry of the device, signal and position."""
        self.pico.write("")
        self.pico.write("telem 1")
        self.pico.write(f"fs {self.g.fs}")
        self.pico.write(f"c {self.g.c:.2f}")
        for m, (x, y, z) in enumerate(self.g.mics_m * 1e3):
            self.pico.write(f"mic {m + 1} {x:.2f} {y:.2f} {z:.2f}")
        for cmd in self.sig_commands():
            self.pico.write(cmd)
        self.pico.write(self.src_command(with_az=True))
        self.t_az_sent = time.monotonic()
        self.dirty.clear()

    def send_dirty(self, now):
        if not self.dirty or now - self.t_sent < SEND_MS / 1e3:
            return
        if self.pico and self.sound:                  # while the sound is off everything is sent on switching it on
            if self.dirty & {"freq", "amp", "phase", "extra"}:
                for cmd in self.sig_commands():
                    self.pico.write(cmd)
            if self.dirty & {"az", "rate", "dist", "el"}:
                self.pico.write(self.src_command(with_az="az" in self.dirty))
                if "az" in self.dirty:
                    self.t_az_sent = now
        self.dirty.clear()
        self.t_sent = now

    def poll_pico(self, now):
        while True:
            try:
                line = self.pico.lines.get_nowait()
            except queue.Empty:
                return
            if line is None:
                self.pico_status = "port closed"
                return
            if line.startswith("S "):
                try:
                    fields = [float(v) for v in line.split()[1:]]
                    az = fields[0]
                except (ValueError, IndexError):
                    continue
                self.t_telem = now
                self.pico_gate = len(fields) < 5 or fields[4] > 0
                if "az" not in self.dirty and now - self.t_az_sent > 0.25:   # else: our change is on its way
                    self.az = az
            elif line.startswith("clk "):
                self.pico_status = line

    def poll_esp(self, now):
        while True:
            try:
                line = self.esp.lines.get_nowait()
            except queue.Empty:
                return
            if line is None:
                continue
            m = re.search(r"src az=(-?\d+) el=(-?\d+) d=([\d.]+)m", line)
            if m:
                self.esp_src = (now, float(m.group(1)), float(m.group(2)), float(m.group(3)))
            m = re.search(r"spk=(\d+)(?: az=(-?\d+))?", line)
            if m:
                self.esp_spk = f"speakers {m.group(1)}" + (f", track az {m.group(2)} deg" if m.group(2) else "")

    # ------------------------------------------------------------------ drawing

    def tick(self):
        now = time.monotonic()
        dt, self.t_last = now - self.t_last, now
        if self.pico:
            self.poll_pico(now)
            live = now - self.t_telem < 0.5
        else:
            live = True
        if self.esp:
            self.poll_esp(now)
        if live and self.sound:
            # simulation: PC clock; with the Pico: its reports (10 per second), advanced at the set speed in between
            self.az = (self.az + self.p.rate_dps * dt) % 360.0
        self.send_dirty(now)

        self.updating = True
        self.vars["az"].set(self.az)
        self.updating = False
        if self.root.focus_get() is not self.entries["az"]:      # do not overwrite what is being typed
            self.show("az", self.az)

        self.trail.append((now, self.az, self.p.el_deg, self.p.dist_m))
        while self.trail and now - self.trail[0][0] > TRAIL_S:
            self.trail.pop(0)
        pos = from_spherical(self.az, self.p.el_deg, self.p.dist_m)
        tau, gain = mic_delays(self.g, pos, self.p.dist_m)
        self.draw_scene(now, pos)
        self.draw_zoom(pos, tau)
        self.draw_waves(now, tau, gain)
        self.draw_text(now, live)
        self.root.after(TICK_MS, self.tick)

    def gate_open(self, now):
        if not self.sound:
            return False
        if self.pico:
            return self.pico_gate
        if self.p.burst_off_ms <= 0:
            return True
        period = (self.p.burst_on_ms + self.p.burst_off_ms) / 1e3
        return (now - self.t_start) % period < self.p.burst_on_ms / 1e3

    def draw_scene(self, now, pos):
        c = self.scene
        c.delete("all")
        half = SCENE_PX / 2
        r_view = max(1.0, math.ceil(self.p.dist_m * math.cos(math.radians(self.p.el_deg)) * 1.25 * 2) / 2)
        k = (half - 34) / r_view

        def xy(x, y):
            return half + x * k, half - y * k

        step = 0.5 if r_view <= 3 else (1.0 if r_view <= 7 else 2.0)
        r = step
        while r <= r_view + 1e-9:
            c.create_oval(*xy(-r, r), *xy(r, -r), outline=GRID)
            c.create_text(half + 4, half - r * k - 7, text=f"{r:g} m", fill=MUTED, font=("TkDefaultFont", 8), anchor="w")
            r += step
        for a in range(0, 360, 30):
            ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
            c.create_line(*xy(0, 0), *xy(r_view * ca, r_view * sa), fill=GRID)
            c.create_text(*xy((r_view + 0.06 * r_view) * ca, (r_view + 0.06 * r_view) * sa), text=f"{a}°",
                          fill=MUTED, font=("TkDefaultFont", 9))
        c.create_text(10, 10, text="Top view (x = azimuth 0°, y = azimuth 90°)", anchor="nw", fill=MUTED,
                      font=("TkDefaultFont", 9))

        # array
        for m in (3, 4, 5):
            c.create_line(*xy(0, 0), *xy(*self.g.mics_m[m][:2]), fill=C_ARM, width=2)
        for m in range(6):
            x, y = xy(*self.g.mics_m[m][:2])
            c.create_oval(x - 3, y - 3, x + 3, y + 3, fill=MIC_COLORS[m], outline="")

        # trail, source
        for i in range(1, len(self.trail)):
            (_, a0, e0, d0), (_, a1, e1, d1) = self.trail[i - 1], self.trail[i]
            if abs((a1 - a0 + 180) % 360 - 180) > 30:
                continue
            p0, p1 = from_spherical(a0, e0, d0), from_spherical(a1, e1, d1)
            c.create_line(*xy(p0[0], p0[1]), *xy(p1[0], p1[1]), fill="#f4b89b", width=3)
        sx, sy = xy(pos[0], pos[1])
        c.create_line(*xy(0, 0), sx, sy, fill=C_SRC, width=1, dash=(4, 3))
        arc = min(0.35 * r_view, 0.6) * k
        c.create_arc(half - arc, half - arc, half + arc, half + arc, start=0, extent=self.az or 0.01, style="arc",
                     outline=C_SRC, width=2)
        open_ = self.gate_open(now)
        if open_:
            for ring in (16, 24):
                c.create_oval(sx - ring, sy - ring, sx + ring, sy + ring, outline="#f4b89b")
        c.create_oval(sx - 9, sy - 9, sx + 9, sy + 9, fill=C_SRC if open_ else "white", outline=C_SRC, width=2)
        if self.p.rate_dps:
            t = math.radians(self.az) + math.copysign(math.pi / 2, self.p.rate_dps)
            c.create_line(sx, sy, sx + 34 * math.cos(t), sy - 34 * math.sin(t), fill=C_SRC, width=2, arrow="last")
        anchor = "w" if sx < half else "e"
        c.create_text(sx + (16 if anchor == "w" else -16), sy - 22, anchor=anchor, fill=INK,
                      font=("TkDefaultFont", 10, "bold"),
                      text=f"az {self.az:.1f}°  d {self.p.dist_m:.2f} m  el {self.p.el_deg:.0f}°")

        if self.esp_src and now - self.esp_src[0] < 3.0:
            _, az, el, dist = self.esp_src
            d = dist if dist > 0 else self.p.dist_m          # direction only: drawn at the true distance
            ex, ey = xy(*from_spherical(az, el, d)[:2])
            c.create_line(*xy(0, 0), ex, ey, fill=C_ESP, dash=(2, 4))
            c.create_rectangle(ex - 8, ey - 8, ex + 8, ey + 8, outline=C_ESP, width=2)
            c.create_text(ex, ey + 20, text="ESP32", fill=C_ESP, font=("TkDefaultFont", 9, "bold"))

    def zoom_xy(self, x, y):
        k = ZOOM_PX / 2 / ZOOM_HALF_M
        return ZOOM_PX / 2 + x * k, ZOOM_PX / 2 - y * k

    def draw_zoom_static(self):
        c = self.zoom
        c.create_text(8, 8, text="Array, top view (mm)", anchor="nw", fill=MUTED, font=("TkDefaultFont", 9))
        for r in (0.1, 0.2, 0.3, 0.4):
            c.create_oval(*self.zoom_xy(-r, r), *self.zoom_xy(r, -r), outline=GRID)
        c.create_line(*self.zoom_xy(-ZOOM_HALF_M, 0), *self.zoom_xy(ZOOM_HALF_M, 0), fill=GRID)
        c.create_line(*self.zoom_xy(0, -ZOOM_HALF_M), *self.zoom_xy(0, ZOOM_HALF_M), fill=GRID)
        for m in (3, 4, 5):
            c.create_line(*self.zoom_xy(0, 0), *self.zoom_xy(*self.g.mics_m[m][:2]), fill=C_ARM, width=2)
        body = [self.zoom_xy(*self.g.mics_m[m][:2]) for m in (0, 1, 2)]
        c.create_polygon(*[v for pt in body for v in pt], outline=C_BODY, fill="", width=1)
        for m in range(6):
            x, y = self.zoom_xy(*self.g.mics_m[m][:2])
            c.create_oval(x - 4, y - 4, x + 4, y + 4, fill=MIC_COLORS[m], outline="")
            mx, my, mz = self.g.mics_m[m] * 1e3
            norm = math.hypot(mx, my) or 1.0
            if m < 3:                                    # body mics sit 35 mm from the centre: labels outside
                lx, ly = x + 30 * mx / norm, y - 22 * my / norm
                c.create_text(lx, ly, text=f"M{m + 1}", fill=MIC_COLORS[m], font=("TkDefaultFont", 9, "bold"))
            else:
                # arm mics: label below the dot (upper arms) or beside it (lower arm), clear of the arm line
                lx, ly = (x, y + 28) if my > 0 else (x + 46, y - 8)
                c.create_text(lx, ly, text=f"M{m + 1}\n({mx:.0f}, {my:.0f})\nz {mz:.0f}", fill=MIC_COLORS[m],
                              font=("TkDefaultFont", 8), justify="center")

    def draw_zoom(self, pos, tau):
        c = self.zoom
        c.delete("dyn")
        d = pos[:2] / (np.linalg.norm(pos[:2]) + 1e-12)
        x0, y0 = self.zoom_xy(*(d * 0.05))
        x1, y1 = self.zoom_xy(*(d * (ZOOM_HALF_M - 0.03)))
        c.create_line(x0, y0, x1, y1, fill=C_SRC, width=2, arrow="last", tags="dyn")
        c.create_text(8, ZOOM_PX - 8, anchor="sw", fill=C_SRC, font=("TkDefaultFont", 9), tags="dyn",
                      text=f"source at az {self.az:.1f}°")

    def draw_waves(self, now, tau, gain):
        c = self.waves
        c.delete("all")
        t_ms, x = triggered_waves(self.p, tau, gain)
        level = 10 ** (self.p.amp_dbfs / 20) if self.gate_open(now) else 0.0
        c.create_text(8, 6, anchor="nw", fill=MUTED, font=("TkDefaultFont", 9),
                      text=f"Mic signals ({t_ms[-1]:.1f} ms) and delay vs centre")
        row = (WAVE_H - 24) / 6
        x0, x1 = 34, ZOOM_PX - 74
        scale = 0.45 * row * min(1.0, level / 0.25) / max(1.0, float(np.max(gain)))
        for m in range(6):
            y = 24 + row * (m + 0.5)
            c.create_text(6, y, text=f"M{m + 1}", anchor="w", fill=MIC_COLORS[m], font=("TkDefaultFont", 9, "bold"))
            c.create_text(ZOOM_PX - 6, y, text=f"{tau[m] * 1e6:+7.0f} µs", anchor="e", fill=INK,
                          font=("TkFixedFont", 9))
            pts = np.empty(2 * x.shape[1])
            pts[0::2] = x0 + (x1 - x0) * t_ms / t_ms[-1]
            pts[1::2] = y - x[m] * scale
            c.create_line(*pts.tolist(), fill=MIC_COLORS[m])

    def draw_text(self, now, live):
        p = self.p
        sig = (f"sweep {p.freq_hz:.0f} -> {p.sweep_to_hz:.0f} Hz in {p.sweep_period_s * 1e3:.0f} ms"
               if p.sweep_period_s > 0 else f"tone {p.freq_hz:.0f} Hz")
        burst = f", bursts {p.burst_on_ms:.0f}/{p.burst_off_ms:.0f} ms" if p.burst_off_ms > 0 else ", continuous"
        lines = [f"source  az {self.az:6.1f} deg   el {p.el_deg:5.1f} deg   dist {p.dist_m:5.2f} m   {p.rate_dps:+6.1f} deg/s",
                 f"signal  {sig}, {p.amp_dbfs:.1f} dBFS, phase {p.phase_deg:.0f} deg{burst}"
                 + ("" if self.sound else "   [sound off]")]
        if self.esp:
            if self.esp_src and now - self.esp_src[0] < 3.0:
                _, az, el, dist = self.esp_src
                err = (az - self.az + 180) % 360 - 180
                lines.append(f"ESP32   az {az:6.0f} deg   el {el:5.0f} deg   dist {dist:5.2f} m   "
                             f"az error {err:+.0f} deg")
            else:
                lines.append(f"ESP32   no source reported   {self.esp_spk}")
        self.readout.configure(text="\n".join(lines))
        if not self.pico:
            status = "Pico not connected: simulation only, nothing is sent."
        elif not live and self.sound:
            status = f"Pico: no telemetry, source frozen ({self.pico_status or 'no status yet'})"
        else:
            status = f"Pico: {self.pico_status}"
        self.status.configure(text=status, fg=MUTED if (live or not self.pico) else C_SRC)

    def close(self):
        if self.pico:
            self.pico.write("telem 0")
            time.sleep(0.05)
            self.pico.close()
        if self.esp:
            self.esp.close()
        self.root.destroy()


def write_wav(path, samples, fs):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(samples.shape[1])
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes(samples.astype("<i2").tobytes())


def main():
    d = LiveParams()
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--az", type=float, default=d.az_deg, help="start azimuth of the source in degrees")
    ap.add_argument("--dist", type=float, default=d.dist_m, help="distance from the array origin in m")
    ap.add_argument("--rate", type=float, default=d.rate_dps, help="rotation speed in degrees per second")
    ap.add_argument("--el", type=float, default=d.el_deg, help="elevation in degrees")
    ap.add_argument("--freq", type=float, default=d.freq_hz, help="frequency in Hz (start of the sweep)")
    ap.add_argument("--amp", type=float, default=d.amp_dbfs, help="amplitude at the array origin in dBFS")
    ap.add_argument("--phase", type=float, default=d.phase_deg, help="phase of the sinusoid in degrees")
    ap.add_argument("--sweep", nargs=2, type=float, metavar=("TO_HZ", "PERIOD_S"),
                    default=[d.sweep_to_hz, d.sweep_period_s], help="triangular sweep freq -> TO_HZ -> freq (PERIOD_S 0 = pure tone)")
    ap.add_argument("--burst", nargs=2, type=float, metavar=("ON_MS", "OFF_MS"),
                    default=[d.burst_on_ms, d.burst_off_ms], help="bursts (OFF_MS 0 = continuous)")
    ap.add_argument("--port", help="USB serial port of the Pico 2 running pico_emu (default: found automatically)")
    ap.add_argument("--no-pico", action="store_true", help="do not look for the Pico: simulation only")
    ap.add_argument("--esp-port", help="UART console port of the ESP32 (shows the source it reports)")
    ap.add_argument("--config", help="daq-detector/main/config.h to take the mic geometry from")
    ap.add_argument("--wiring", action="store_true", help="open the wiring diagram (Pico 2 <-> ESP32) at start")
    ap.add_argument("--wav", help="no window: write the signal of the six mics to this WAV file")
    ap.add_argument("--seconds", type=float, default=10.0, help="length of the WAV (with --wav)")
    args = ap.parse_args()

    geometry = load_geometry(args.config)
    params = LiveParams(az_deg=args.az, el_deg=args.el, dist_m=args.dist, rate_dps=args.rate, freq_hz=args.freq,
                        amp_dbfs=args.amp, phase_deg=args.phase, sweep_to_hz=args.sweep[0],
                        sweep_period_s=args.sweep[1], burst_on_ms=args.burst[0], burst_off_ms=args.burst[1])
    if args.wav:
        write_wav(args.wav, render_live(geometry, params, args.seconds), geometry.fs)
        print(f"{args.wav}: {args.seconds:.1f} s, source az {args.az:g} deg + {args.rate:g} deg/s, "
              f"el {args.el:g} deg, dist {args.dist:g} m")
        return

    import tkinter as tk
    from tkinter import ttk
    port = args.port
    if not port and not args.no_pico:
        port = find_pico()
        if port:
            print(f"Pico found on {port}")
        else:
            print("Pico not connected: no serial port of a Raspberry Pi Pico found "
                  "(plug it in without BOOTSEL, with the pico_emu firmware). Running as a simulation only.")
    pico = open_port(port, "--port") if port else None
    esp = open_port(args.esp_port, "--esp-port") if args.esp_port else None
    root = tk.Tk()
    app = App(root, tk, ttk, geometry, params, pico, esp, load_pdm_pins(args.config))
    if args.wiring:
        open_wiring(root, tk, app.pdm_pins)
    root.mainloop()


if __name__ == "__main__":
    main()

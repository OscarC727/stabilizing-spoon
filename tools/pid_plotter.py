"""
Live PID plotter for the ESP32 gimbal.

Reads lines like:
  Target:0,Roll:1.23,Pitch:-0.45,RollErr:-1.23,PitchErr:0.45
and draws Roll and Pitch against the target, with the error shaded.

Usage:
  pip install pyserial matplotlib
  python pid_plotter.py COM3            (Windows)
  python pid_plotter.py /dev/cu.usbserial-XXXX   (Mac)
  python pid_plotter.py /dev/ttyUSB0    (Linux)

Close the Arduino Serial Monitor / Plotter first, or the port will be busy.
"""

import sys
import time
import collections

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = 115200
WINDOW_S = 10      # seconds of history shown
YLIM = 45          # fixed y-axis range in degrees (+/-), so the scale doesn't jump around
MAXPTS = 3000

ser = serial.Serial(PORT, BAUD, timeout=0)
t0 = time.time()

t = collections.deque(maxlen=MAXPTS)
data = {k: collections.deque(maxlen=MAXPTS) for k in ("Target", "Roll", "Pitch")}
rx_buf = ""


def read_serial():
    """Pull everything waiting on the port and parse complete lines."""
    global rx_buf
    rx_buf += ser.read(ser.in_waiting or 1).decode(errors="ignore")
    lines = rx_buf.split("\n")
    rx_buf = lines.pop()  # keep any partial line for next time
    for line in lines:
        try:
            fields = dict(p.split(":") for p in line.strip().split(","))
            sample = {k: float(fields[k]) for k in data}
        except (ValueError, KeyError):
            continue  # skip startup messages / garbled lines
        t.append(time.time() - t0)
        for k in data:
            data[k].append(sample[k])


# ---- Figure setup ----
fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
fig.suptitle("Gimbal PID: angle vs target")

panels = {}
for ax, axis_name, color in zip(axes, ("Roll", "Pitch"), ("tab:blue", "tab:orange")):
    target_line, = ax.plot([], [], "k--", lw=1.5, label="Target")
    angle_line, = ax.plot([], [], color=color, lw=2, label=f"{axis_name} angle")
    ax.set_ylim(-YLIM, YLIM)
    ax.set_ylabel("degrees")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    readout = ax.text(0.99, 0.95, "", transform=ax.transAxes, ha="right", va="top",
                      family="monospace", fontsize=11,
                      bbox=dict(boxstyle="round", fc="white", alpha=0.85))
    panels[axis_name] = dict(ax=ax, target=target_line, angle=angle_line,
                             readout=readout, fill=None, color=color)
axes[-1].set_xlabel("time (s)")


def update(_frame):
    read_serial()
    if not t:
        return

    ts = list(t)
    tgt = list(data["Target"])
    x_end = ts[-1]
    x_start = max(0, x_end - WINDOW_S)

    for name, p in panels.items():
        ang = list(data[name])
        p["target"].set_data(ts, tgt)
        p["angle"].set_data(ts, ang)

        # Shade the gap between angle and target = the error
        if p["fill"] is not None:
            p["fill"].remove()
        p["fill"] = p["ax"].fill_between(ts, tgt, ang, color=p["color"], alpha=0.2)

        # Stats over the visible window
        visible = [a - g for tt, a, g in zip(ts, ang, tgt) if tt >= x_start]
        err_now = tgt[-1] - ang[-1]
        err_max = max(abs(e) for e in visible)
        err_rms = (sum(e * e for e in visible) / len(visible)) ** 0.5
        p["readout"].set_text(f"error now: {err_now:+6.2f} deg\n"
                              f"max ({WINDOW_S}s): {err_max:6.2f} deg\n"
                              f"RMS ({WINDOW_S}s): {err_rms:6.2f} deg")

        p["ax"].set_xlim(x_start, max(x_end, WINDOW_S))


anim = FuncAnimation(fig, update, interval=30, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()

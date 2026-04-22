import serial
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

PORT = "COM3"      # <-- change to your STM32 VCP COM port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

N = 600  # points on screen
xs = deque(maxlen=N)
ys = deque(maxlen=N)

def moving_average(y, w=11):
    """Simple smoothing to make derivative usable."""
    y = np.asarray(y, dtype=float)
    if y.size < w:
        return y
    kernel = np.ones(w) / w
    return np.convolve(y, kernel, mode="same")

plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

line_v,  = ax1.plot([], [], lw=1)
line_dv, = ax2.plot([], [], lw=1)

ax1.set_ylabel("Voltage (V)")
ax2.set_ylabel("dV/dIndex")
ax2.set_xlabel("Sample index")
ax1.grid(True)
ax2.grid(True)

# Optional: set a fixed voltage range
ax1.set_ylim(0, 3.3)

while True:
    s = ser.readline().decode(errors="ignore").strip()
    if not s:
        continue

    # Expect "index,voltage"
    try:
        k_str, v_str = s.split(",", 1)
        k = int(k_str)
        v = float(v_str)
    except ValueError:
        continue

    xs.append(k)
    ys.append(v)

    if len(xs) < 20:
        continue

    x = np.array(xs, dtype=float)
    y = np.array(ys, dtype=float)

    # Smooth voltage to reduce noise amplification in derivative
    y_s = moving_average(y, w=11)

    # Derivative (numerical)
    dy_dx = np.gradient(y_s, x)

    # Update plots
    line_v.set_data(x, y_s)
    line_dv.set_data(x, dy_dx)

    ax1.relim()
    ax1.autoscale_view(scalex=False, scaley=True)

    ax2.relim()
    ax2.autoscale_view(scalex=False, scaley=True)

    # Keep x-limits stable to the incoming window
    ax2.set_xlim(x.min(), x.max())

    fig.canvas.draw()
    fig.canvas.flush_events()

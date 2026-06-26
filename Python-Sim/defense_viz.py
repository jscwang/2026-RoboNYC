#!/usr/bin/env python3
"""
Live defense visualizer for Robot-5-Test-Defense.

Reads the serial feed from printDefenseViz() in the firmware (one line per loop):

  V,wlAngle,whiteDist,ballAng,numClusters,slope,cornerness,state,driveAng,speed,<48 isWhite 0/1>

  state: 0 straight  1 push-in  2 retreat  3 back-out  4 recover(off-line)  5 hold(off-line)

and draws the robot's own view live -- the 48 sensors at their real coordinates
(white when over the line), the white line it sees, the ball direction, the
intended drive, and every computed value -- the same readout as defense_sim.py's
right panel (there's no world/field view live since the robot has no global pose).

Usage:   python3 defense_viz.py <serial_port> [baud=115200]
  e.g.   python3 defense_viz.py /dev/cu.usbmodem1234
         python3 defense_viz.py COM5 115200

Deps:  pip install pyserial matplotlib
"""
import sys
import math
import threading
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

plt.rcParams.update({"figure.facecolor": "#faf6ee", "axes.edgecolor": "#b9b2a4"})
SENS_OFF, SENS_ON = "#544d45", "#ffffff"
BALL_C, DRIVE_C, PANEL_BG, READ_C = "#e2674a", "#ef8a3c", "#f4efe4", "#2e86c1"

# Sensor positions (cm from the ring centroid) from BASE_PLATE.kicad_pcb (2026-06-15 board); kept in
# sync with realX/realY in Robot6-Organized/globals.cpp. Index i = SEN_i channel / sheet SENSOR_i;
# +X right, +Y forward. Real distances -- the ring is elliptical (~7.06 cm front/back, ~9.05 cm sides).
REAL_X = [0.959,2.009,3.159,3.934,4.634,5.334,6.034,6.784,7.834,8.659,8.909,8.959,8.934,8.659,7.734,6.784,6.034,5.309,4.659,3.909,3.134,1.984,0.959,0.009,-0.941,-1.991,-3.141,-3.941,-4.666,-5.366,-6.041,-6.841,-7.791,-8.741,-8.941,-9.016,-8.891,-8.641,-7.691,-6.766,-6.041,-5.341,-4.616,-3.866,-3.141,-1.966,-0.941,0.009]
REAL_Y = [-7.218,-7.443,-7.543,-6.793,-6.043,-5.268,-4.618,-3.918,-3.243,-2.318,-1.193,0.007,1.157,2.332,3.232,3.932,4.632,5.332,6.057,6.757,7.632,7.432,7.157,7.057,7.107,7.432,7.607,6.832,6.057,5.382,4.632,3.932,3.207,2.332,1.157,0.007,-1.193,-2.318,-3.218,-3.918,-4.643,-5.343,-6.018,-6.718,-7.593,-7.393,-7.243,-7.218]
N = 48
RING_R = sum(math.hypot(x, y) for x, y in zip(REAL_X, REAL_Y)) / N  # mean sensor radius (cm), from real positions
LIM = 1.3 * max(max(abs(x) for x in REAL_X), max(abs(y) for y in REAL_Y))  # view half-extent (cm)
SENS_R = 0.07 * RING_R  # sensor dot radius (cm)
STATE = {0: "on the line / straight", 1: "corner PUSH-IN", 2: "corner RETREAT",
         3: "corner BACK-OUT", 4: "OFF LINE -> recover", 5: "OFF LINE -> hold"}

latest = {"wl": 0.0, "dist": 0.0, "ball": 0.0, "nc": 0, "slope": 0.0, "corner": 0.0,
          "state": 0, "drive": 0.0, "speed": 0.0, "white": [False] * N, "ok": False}
lock = threading.Lock()


def reader(port, baud):
    import serial
    ser = serial.Serial(port, baud, timeout=1)
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            continue
        if not line.startswith("V,"):
            continue
        p = line.split(",")
        if len(p) != 11 or len(p[10]) < N:
            continue
        try:
            f = dict(wl=float(p[1]), dist=float(p[2]), ball=float(p[3]), nc=int(p[4]),
                     slope=float(p[5]), corner=float(p[6]), state=int(p[7]),
                     drive=float(p[8]), speed=float(p[9]),
                     white=[ch == "1" for ch in p[10][:N]], ok=True)
        except ValueError:
            continue
        with lock:
            latest.update(f)


def rf(deg):
    r = math.radians(deg)
    return math.sin(r), math.cos(r)


def main():
    if len(sys.argv) < 2:
        print("usage: python3 defense_viz.py <serial_port> [baud=115200]")
        sys.exit(1)
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    threading.Thread(target=reader, args=(port, baud), daemon=True).start()

    fig = plt.figure(figsize=(11, 7.4), dpi=120)
    ax = fig.add_axes([0.04, 0.07, 0.60, 0.86])
    ax.set_aspect("equal"); ax.set_xlim(-LIM, LIM); ax.set_ylim(-LIM, LIM)
    ax.set_facecolor(PANEL_BG)
    ax.set_title("LIVE robot view (defense)", fontsize=13, fontweight="bold", color="#3a352e")
    ax.tick_params(labelsize=8, colors="#6b6457")
    ax.axhline(0, color="#e3dccd", lw=.8); ax.axvline(0, color="#e3dccd", lw=.8)
    circles = [mpatches.Circle((REAL_X[i], REAL_Y[i]), SENS_R,
               facecolor=SENS_OFF, edgecolor="#6b6457", lw=.7) for i in range(N)]
    for c in circles:
        ax.add_patch(c)
    read_line, = ax.plot([], [], color=READ_C, lw=3, solid_capstyle="round")
    ball_dot, = ax.plot([], [], "o", color=BALL_C, ms=10, markeredgecolor="#9c3d28")
    drive_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                              arrowprops=dict(arrowstyle="-|>", color=DRIVE_C, lw=3.5))
    stats = fig.text(0.66, 0.90, "waiting for serial...", family="monospace",
                     fontsize=10.5, va="top", color="#3a352e")

    def update(_):
        with lock:
            f = dict(latest)
        if not f["ok"]:
            return
        for i, c in enumerate(circles):
            c.set_facecolor(SENS_ON if f["white"][i] else SENS_OFF)
        if f["nc"] > 0:
            nx, ny = rf(f["wl"]); d = f["dist"] * RING_R
            fx, fy = d * nx, d * ny; tx, ty = ny, -nx; L = LIM
            read_line.set_data([fx - L * tx, fx + L * tx], [fy - L * ty, fy + L * ty])
        else:
            read_line.set_data([], [])
        bx, by = rf(f["ball"])
        ball_dot.set_data([0.9 * RING_R * bx], [0.9 * RING_R * by])
        ddx, ddy = rf(f["drive"]); amp = 1.1 * RING_R * max(f["speed"], 0.04)
        drive_arrow.set_position((0, 0)); drive_arrow.xy = (amp * ddx, amp * ddy)
        stats.set_text(
            f"INPUTS\n"
            f"  ball_ang   = {f['ball']:6.1f}  ({'RIGHT' if f['ball']<180 else 'LEFT'})\n"
            f"  #clusters  = {f['nc']}\n"
            f"  whiteDist  = {f['dist']:5.2f}\n"
            f"  wlAngle    = {f['wl']:6.1f}\n\n"
            f"CORNER GATE\n"
            f"  slope      = {f['slope']:+6.2f}\n"
            f"  cornerness = {f['corner']:5.2f}\n"
            f"  -> {STATE.get(f['state'], '?')}\n\n"
            f"OUTPUT\n"
            f"  drive_ang  = {f['drive']:6.1f}\n"
            f"  robot_speed= {f['speed']:5.2f}")
        return circles + [read_line, ball_dot, drive_arrow, stats]

    _anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()

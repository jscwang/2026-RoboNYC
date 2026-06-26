#!/usr/bin/env python3
"""
Interactive goalkeeper defense simulator for Robot-5-Test-Defense.

Drive the robot around a penalty-box line (80 cm front, curving back 25 cm on
both sides, 15 cm rounded corners) and watch the REAL clustering + whiteMove()
logic compute live.

  LEFT panel  : the field/world -- penalty line, robot + its 48 sensors
                (white when over the line), the ball, and the intended drive.
  RIGHT panel : the robot's own view -- 48 sensors at real coords, the white
                line it thinks it sees, the intended drive, and every
                intermediate calculation as text.

Controls:
  Arrow keys ........ move the robot (world frame; robot stays facing North)
  Mouse click ....... place the ball anywhere (ball_ang updates as you move)
  [ / ] ............. shrink / grow the robot step size
  r ................. reset robot to center

Deps: pip install matplotlib numpy
Run:  python3 defense_sim.py
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

plt.rcParams.update({
    "font.size": 10,
    "axes.titlesize": 12,
    "axes.titleweight": "bold",
    "figure.facecolor": "#faf6ee",
    "axes.edgecolor": "#b9b2a4",
})

# ---- warm color palette ------------------------------------------------------
FIELD    = "#bcd9a4"   # soft field green
OOB      = "#e7e1d4"   # out of bounds (warm light gray)
LINE_C   = "#ffffff"   # white line
SENS_OFF = "#544d45"   # warm charcoal (sensor not over line)
SENS_ON  = "#ffffff"   # white (sensor over line)
BALL_C   = "#e2674a"   # warm red-orange
DRIVE_C  = "#ef8a3c"   # warm orange
PANEL_BG = "#f4efe4"   # warm cream (robot view)
READ_C   = "#2e86c1"   # read-line blue
ROBOT_FC = "#000000"   # robot body (drawn translucent)

# ---- real sensor coordinates (cm) from BASE_PLATE.kicad_pcb; index i = SEN_i / sheet SENSOR_i --
REAL_X = np.array([0.959,2.009,3.159,3.934,4.634,5.334,6.034,6.784,7.834,8.659,8.909,8.959,8.934,8.659,7.734,6.784,6.034,5.309,4.659,3.909,3.134,1.984,0.959,0.009,-0.941,-1.991,-3.141,-3.941,-4.666,-5.366,-6.041,-6.841,-7.791,-8.741,-8.941,-9.016,-8.891,-8.641,-7.691,-6.766,-6.041,-5.341,-4.616,-3.866,-3.141,-1.966,-0.941,0.009])
REAL_Y = np.array([-7.218,-7.443,-7.543,-6.793,-6.043,-5.268,-4.618,-3.918,-3.243,-2.318,-1.193,0.007,1.157,2.332,3.232,3.932,4.632,5.332,6.057,6.757,7.632,7.432,7.157,7.057,7.107,7.432,7.607,6.832,6.057,5.382,4.632,3.932,3.207,2.332,1.157,0.007,-1.193,-2.318,-3.218,-3.918,-4.643,-5.343,-6.018,-6.718,-7.593,-7.393,-7.243,-7.218])
N = 48
RING_R = float(np.hypot(REAL_X, REAL_Y).mean())  # mean sensor radius (cm) from real positions -- scales whiteDist
ROBOT_R = 11.0         # robot body radius (cm) -- 22 cm diameter
PHYS = list(range(1, N+1))  # new board: index already runs around the ring (was a permutation)
HALFWIDTH = 1.0        # cm: half the 2 cm line width

def rad(d): return math.radians(d)

# ---- penalty-box line: 80 cm front + 25 cm sides + 15 cm rounded corners -----
# Penalty box: 80 cm wide x 25 cm deep overall, 15 cm rounded corners.
HALF_W, DEPTH, RC = 40.0, 25.0, 15.0   # total width/2, depth, corner radius
SX = HALF_W - RC                        # front straight half-width (25 cm)
FIELD_W, FIELD_H = 158.0, 110.0         # green field size (cm)
FIELD_X0, FIELD_Y0 = -FIELD_W/2, -DEPTH # field lower-left; goal line at the box back
def build_line():
    # One continuous path (no diagonal): left side up -> left corner -> front
    # -> right corner -> right side down.
    xs, ys = [], []
    for y in np.linspace(-DEPTH, -RC, 30): xs.append(-HALF_W); ys.append(y)         # left side
    for t in np.linspace(180, 90, 40):                                              # left corner
        xs.append(-SX + RC*math.cos(rad(t))); ys.append(-RC + RC*math.sin(rad(t)))
    for x in np.linspace(-SX, SX, 120): xs.append(x); ys.append(0.0)                # front (50 cm)
    for t in np.linspace(90, 0, 40):                                                # right corner
        xs.append(SX + RC*math.cos(rad(t))); ys.append(-RC + RC*math.sin(rad(t)))
    for y in np.linspace(-RC, -DEPTH, 30): xs.append(HALF_W); ys.append(y)          # right side
    return np.array(xs), np.array(ys)
LINE_X, LINE_Y = build_line()

def sensors_white(rx, ry):
    """World positions of the 48 sensors -> True where within HALFWIDTH of line."""
    sx = rx + REAL_X; sy = ry + REAL_Y
    dx = sx[:, None] - LINE_X[None, :]
    dy = sy[:, None] - LINE_Y[None, :]
    dmin = np.sqrt(dx*dx + dy*dy).min(axis=1)
    return dmin < HALFWIDTH

# ---- getWhiteAngClustered: white-line angle + cross-track distance ----
def cluster(isW):
    return 0.0, 0.0, 0

# ---- whiteMove: line-hug steering, returns internals for display ----
def whiteMove(wl, wd, ball):
    return dict(tangent=0.0, offset=0.0, drive=0.0, speed=0.0, slope=0.0,
                cornerness=0.0, note="", ballRight=ball < 180)

def compute(rx, ry, ball_world, lastWhiteAng=0.0):
    bx, by = ball_world
    isW = sensors_white(rx, ry)
    ball_ang = math.degrees(math.atan2(bx-rx, by-ry)) % 360
    wl, wd, nc = cluster(isW)
    if nc > 0:
        m = whiteMove(wl, wd, ball_ang)
    else:
        # Pushed OFF the line: drive back toward the last white-line direction
        # (mirrors defenseMain(): recover toward lastWhiteAng at 0.4; the firmware
        # then holds after 2 s -- the sim always shows the recovery heading).
        m = dict(tangent=0, offset=0, drive=lastWhiteAng, speed=0.40, slope=0, cornerness=0,
                 note=f"OFF LINE -> recover toward lastWhiteAng ({lastWhiteAng:.0f})",
                 ballRight=ball_ang < 180)
    return isW, ball_ang, wl, wd, nc, m

# ============================ rendering ======================================
def render(axW, axR, rx, ry, ball_world, step, lastWhiteAng=0.0):
    bx, by = ball_world
    isW, ball_ang, wl, wd, nc, m = compute(rx, ry, ball_world, lastWhiteAng)
    new_last = wl if nc > 0 else lastWhiteAng   # remember the line direction for off-line recovery

    # ---------- WORLD / FIELD panel ----------
    axW.clear()
    axW.set_facecolor(OOB)
    axW.set_aspect("equal")
    axW.set_xlim(FIELD_X0-9, FIELD_X0+FIELD_W+9); axW.set_ylim(FIELD_Y0-7, FIELD_Y0+FIELD_H+7)
    axW.set_title("FIELD  (158 x 110 cm, goal below)")
    axW.tick_params(labelsize=8, colors="#6b6457")
    axW.add_patch(mpatches.Rectangle((FIELD_X0, FIELD_Y0), FIELD_W, FIELD_H,
                  facecolor=FIELD, edgecolor="#8aa878", lw=1.5, zorder=0))
    axW.plot(LINE_X, LINE_Y, color=LINE_C, lw=3.5, solid_capstyle="round", zorder=2)
    axW.add_patch(mpatches.Circle((rx, ry), ROBOT_R, facecolor=ROBOT_FC, alpha=0.06, zorder=3))
    axW.add_patch(mpatches.Circle((rx, ry), ROBOT_R, fill=False, ec="#5f5a50", lw=1.4, zorder=4))
    axW.scatter(rx+REAL_X, ry+REAL_Y, s=30, zorder=5,
                c=[SENS_ON if w else SENS_OFF for w in isW], edgecolors="#6b6457", linewidths=0.6)
    axW.plot(rx, ry, "+", color="#3a352e", ms=10, zorder=6)
    axW.plot(bx, by, "o", color=BALL_C, ms=13, zorder=7, markeredgecolor="#9c3d28")
    axW.text(bx, by+2.2, "ball", ha="center", color="#9c3d28", fontsize=9, fontweight="bold")
    if m["drive"] is not None and m["speed"] > 0.01:
        ddx, ddy = math.sin(rad(m["drive"])), math.cos(rad(m["drive"]))
        amp = 24*m["speed"]
        axW.annotate("", xy=(rx+amp*ddx, ry+amp*ddy), xytext=(rx, ry), zorder=8,
                     arrowprops=dict(arrowstyle="-|>", color=DRIVE_C, lw=3.5))
    axW.text(FIELD_X0+2, FIELD_Y0+FIELD_H-3, f"step = {step:.1f} cm", fontsize=8.5, color="#6b6457")

    # ---------- ROBOT VIEW panel ----------
    axR.clear()
    axR.set_facecolor(PANEL_BG)
    axR.set_aspect("equal"); axR.set_xlim(-13, 13); axR.set_ylim(-13, 13)
    axR.set_title("ROBOT VIEW  (sensors + live math)")
    axR.tick_params(labelsize=8, colors="#6b6457")
    axR.axhline(0, color="#e3dccd", lw=.8); axR.axvline(0, color="#e3dccd", lw=.8)
    for i in range(N):
        axR.add_patch(mpatches.Circle((REAL_X[i], REAL_Y[i]), 0.72,
                      facecolor=SENS_ON if isW[i] else SENS_OFF, edgecolor="#6b6457", lw=.7))
    if nc > 0:
        nx, ny = math.sin(rad(wl)), math.cos(rad(wl)); d = wd*RING_R
        fx, fy = d*nx, d*ny; tx, ty = ny, -nx; L = 13
        axR.plot([fx-L*tx, fx+L*tx], [fy-L*ty, fy+L*ty], color=READ_C, lw=3, solid_capstyle="round")
    if m["drive"] is not None and m["speed"] > 0.01:   # drive arrow (incl. off-line recovery)
        ddx, ddy = math.sin(rad(m["drive"])), math.cos(rad(m["drive"]))
        amp = 11*max(m["speed"], 0.04)
        axR.annotate("", xy=(amp*ddx, amp*ddy), xytext=(0, 0),
                     arrowprops=dict(arrowstyle="-|>", color=DRIVE_C, lw=3.5))
    bdx, bdy = math.sin(rad(ball_ang)), math.cos(rad(ball_ang))
    axR.plot(9*bdx, 9*bdy, "o", color=BALL_C, ms=10, markeredgecolor="#9c3d28")

    drive_s = f"{m['drive']:.0f}" if m["drive"] is not None else "--"
    txt = (f"INPUTS\n"
           f"  ball_ang   = {ball_ang:6.1f}  ({'RIGHT' if m['ballRight'] else 'LEFT'})\n"
           f"  #clusters  = {nc}\n"
           f"  whiteDist  = {wd:5.2f}\n"
           f"  wlAngle    = {wl:6.1f}\n\n"
           f"STEERING\n"
           f"  tangent    = {m['tangent']:6.1f}\n"
           f"  restore off= {m['offset']:5.1f} deg\n"
           f"  drive_ang  = {drive_s}\n\n"
           f"CORNER GATE\n"
           f"  slope      = {m['slope']:+6.2f}\n"
           f"  cornerness = {m['cornerness']:5.2f}\n"
           f"  -> {m['note']}\n\n"
           f"OUTPUT\n"
           f"  robot_speed= {m['speed']:5.2f}")
    axR.text(14.2, 12.5, txt, family="monospace", fontsize=9.5, va="top", color="#3a352e")
    return new_last

def main():
    state = {"rx": 0.0, "ry": -4.0, "ball": (16.0, 14.0), "step": 2.0, "lastWhiteAng": 0.0}
    fig, (axW, axR) = plt.subplots(1, 2, figsize=(15.5, 7.6), dpi=120)
    fig.suptitle("Defense simulator   |   arrows: move robot    click: place ball    [ ]: step size    r: reset",
                 fontsize=12, fontweight="bold", color="#3a352e")

    def on_key(e):
        s = state["step"]
        if e.key == "up": state["ry"] += s
        elif e.key == "down": state["ry"] -= s
        elif e.key == "left": state["rx"] -= s
        elif e.key == "right": state["rx"] += s
        elif e.key == "[": state["step"] = max(0.5, state["step"]-0.5)
        elif e.key == "]": state["step"] = min(6.0, state["step"]+0.5)
        elif e.key == "r": state["rx"], state["ry"] = 0.0, -4.0

    def on_click(e):
        if e.inaxes is axW and e.xdata is not None:
            state["ball"] = (e.xdata, e.ydata)

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("button_press_event", on_click)

    def draw(_):
        state["lastWhiteAng"] = render(axW, axR, state["rx"], state["ry"],
                                       state["ball"], state["step"], state["lastWhiteAng"])

    _anim = FuncAnimation(fig, draw, interval=60, blit=False, cache_frame_data=False)
    fig.subplots_adjust(left=0.045, right=0.78, top=0.91, bottom=0.07, wspace=0.16)
    plt.show()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Offline demo of the defense module: feeds synthetic line readings (a line that
curves from horizontal toward vertical = a corner) through the REAL clustering +
whiteMove() logic and renders each frame like defense_viz.py.
Run: python3 defense_viz_demo.py   ->   writes defense_demo.png
"""
import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

REAL_X=[0.959,2.009,3.159,3.934,4.634,5.334,6.034,6.784,7.834,8.659,8.909,8.959,8.934,8.659,7.734,6.784,6.034,5.309,4.659,3.909,3.134,1.984,0.959,0.009,-0.941,-1.991,-3.141,-3.941,-4.666,-5.366,-6.041,-6.841,-7.791,-8.741,-8.941,-9.016,-8.891,-8.641,-7.691,-6.766,-6.041,-5.341,-4.616,-3.866,-3.141,-1.966,-0.941,0.009]
REAL_Y=[-7.218,-7.443,-7.543,-6.793,-6.043,-5.268,-4.618,-3.918,-3.243,-2.318,-1.193,0.007,1.157,2.332,3.232,3.932,4.632,5.332,6.057,6.757,7.632,7.432,7.157,7.057,7.107,7.432,7.607,6.832,6.057,5.382,4.632,3.932,3.207,2.332,1.157,0.007,-1.193,-2.318,-3.218,-3.918,-4.643,-5.343,-6.018,-6.718,-7.593,-7.393,-7.243,-7.218]
# positions: cm from ring centroid (BASE_PLATE.kicad_pcb 2026-06-15); index i = SEN_i / sheet SENSOR_i
N=48; RING_R=sum(math.hypot(x,y) for x,y in zip(REAL_X,REAL_Y))/N  # mean sensor radius (cm), derived
PHYS=list(range(1,N+1))  # new board: index already runs around the ring (was a permutation)
def rad(d): return math.radians(d)

# ---- getWhiteAngClustered: cluster -> unit vectors -> sum ----
def cluster(isW):
    return 0.0, 0.0, 0

# ---- whiteMove ----
def whiteMove(wl, wd, ball):
    return 0.0, 0.0, 0.0, ""

# ---- synthesize which sensors are white for a straight line (normal=wl, offset d) ----
def gen_white(wl, d_cm, hw=1.4):
    nx,ny=math.sin(rad(wl)),math.cos(rad(wl))
    return [abs(REAL_X[i]*nx+REAL_Y[i]*ny - d_cm) < hw for i in range(N)]

# scenario: line bends 180 -> 90 (a corner), ball on the right, + edge cases
scenes=[
 ("1. straight, ball R",       180, 2.6,  90),
 ("2. entering corner",        160, 2.6,  90),
 ("3. corner near (slowing)",  150, 2.6,  90),
 ("4. corner midpoint (STOP)", 135, 2.6,  90),
 ("5. deep corner (stopped)",  115, 2.6,  90),
 ("6. EDGE ball flips L (retreat)",135,2.6, 270),
 ("7. EDGE ball centered (hold)",180,2.6,   8),
 ("8. EDGE drifted off line",  180, 8.4,  90),
 ("9. EDGE line lost",         180, 13.0, 90),
]

fig,axes=plt.subplots(3,3,figsize=(16,16)); axes=axes.flatten()
for ax,(label,wl_t,d_t,ball) in zip(axes,scenes):
    isW=gen_white(wl_t,d_t)
    wl,wd,nc=cluster(isW)
    drive,spd,slope,note=whiteMove(wl,wd,ball) if nc>0 else (0,0,0,"NO LINE -> defenseMain recovery")
    ax.set_aspect("equal"); ax.set_xlim(-12,12); ax.set_ylim(-12,12)
    ax.axhline(0,color="#eee",lw=.5); ax.axvline(0,color="#eee",lw=.5)
    for i in range(N):
        ax.add_patch(mpatches.Circle((REAL_X[i],REAL_Y[i]),0.7,
            facecolor="white" if isW[i] else "#222",edgecolor="#888",lw=.7))
    # read line
    if nc>0:
        nx,ny=math.sin(rad(wl)),math.cos(rad(wl)); d=wd*RING_R
        fx,fy=d*nx,d*ny; tx,ty=ny,-nx; L=13
        ax.plot([fx-L*tx,fx+L*tx],[fy-L*ty,fy+L*ty],color="cyan",lw=3)
        # drive arrow (length ~ speed)
        dx,dy=math.sin(rad(drive)),math.cos(rad(drive)); amp=11*max(spd,0.04)
        ax.annotate("",xy=(amp*dx,amp*dy),xytext=(0,0),
            arrowprops=dict(arrowstyle="-|>",color="orange",lw=3))
    ball_dx,ball_dy=math.sin(rad(ball)),math.cos(rad(ball))
    ax.plot([9*ball_dx],[9*ball_dy],marker="o",color="red",ms=10)  # ball direction marker
    ax.set_title(f"{label}\nwl={wl:.0f} slope={slope:+.2f} dist={wd:.2f} | drive={drive:.0f} spd={spd:.2f}\n{note}",fontsize=10)
fig.suptitle("Defense module demo: line curving horizontal->vertical (red dot = ball dir, cyan = read line, orange = intended drive)",fontsize=13)
fig.tight_layout(rect=[0,0,1,0.97])
out=os.path.join(os.path.dirname(os.path.abspath(__file__)),"defense_demo.png")
fig.savefig(out,dpi=100); print("wrote",out)

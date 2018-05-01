import json
import sys
from matplotlib import pyplot as plt
from matplotlib import animation
import os
import csv
from shapely.geometry.polygon import LinearRing, Polygon
from descartes import PolygonPatch
import numpy as np

jsn = json.load(open(sys.argv[1]))

save = None
if (len(sys.argv) > 2 and sys.argv[2] == "save"):
    save = True
else:
    save = False
colors = ['b', 'g', 'r', 'c', 'm', 'k']

fig = plt.figure()
ax = plt.axes(xlim=(-10,10), ylim=(-10,10))
ax.set_aspect('equal')
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
trajectories = []
to_animate = []

polygon_patches = []
if("obstacles" in jsn):
    for obs in jsn["obstacles"]:
        poly = Polygon(obs)
        polypatch = PolygonPatch(poly, color='yellow', zorder = 5)
        ax.add_patch(polypatch)
        to_animate.append(polypatch)


circles = []
voronois = []
plans = []
originals = []
controlpoints = []

for i in range(jsn["number_of_robots"]):
    orig = ax.plot(jsn["originals"][i]["x"], jsn["originals"][i]["y"], linestyle="dashed", lw=2, zorder = 10, color=colors[i], alpha = 0.8)[0]
    originals.append(orig)
    to_animate.append(orig)

for i in range(jsn["number_of_robots"]):
    x = jsn["planned_trajs"][0][i]["x"]
    y = jsn["planned_trajs"][0][i]["x"]
    tr = ax.plot(x, y, lw=2, zorder = 10, color=colors[i], alpha=0.4)[0]
    plans.append(tr)
    to_animate.append(tr)

for i in range(jsn["number_of_robots"]):
    pts = jsn["controlpoints"][0][i]
    cpts = []
    print(len(pts))
    for pt in pts:
        ptd = ax.plot([pt[0]], [pt[1]], zorder = 20, color=colors[i], alpha = 0.8, marker='o')[0]
        cpts.append(ptd)
        to_animate.append(ptd)
    controlpoints.append(cpts)

for i in range(jsn["number_of_robots"]):
    atraj = ax.plot([], [], lw=2, zorder=0, color = colors[i])[0]
    trajectories.append(atraj)
    to_animate.append(atraj)

    circle = plt.Circle((jsn["points"][0][i][0],jsn["points"][0][i][1]), radius=0.15, fc=colors[i], lw=0, alpha=0.2)
    circles.append(circle)
    to_animate.append(circle)
    ax.add_artist(circle)

def genvoroxy(voronoi):
    a = voronoi[0]
    b = voronoi[1]
    c = voronoi[2]
    xc = np.arange(-5,5,0.01).tolist()
    yc = []
    for xx in xc:
        yc.append((c-a*xx)/b)
    return (xc,yc)
if "voronois" in jsn:
    for i in range(jsn["number_of_robots"]):
        voronois.append([])
        for v in jsn["voronois"][0][i]:
            (x,y) = genvoroxy(v)
            v = ax.plot(x, y, lw=2, zorder=7, color=colors[i])[0]
            voronois[i].append(v)
            to_animate.append(v)

to_animate.append(time_text)
x = []
y = []

def init():
    global trajectories
    global to_animate
    global time_text
    global x
    global y
    global voronois
    global controlpoints
    x = []
    y = []
    for atraj in trajectories:
        atraj.set_data([],[])
        x.append([])
        y.append([])
    time_text.set_text('')
    if "voronois" in jsn:
        for i in range(jsn["number_of_robots"]):
            j = 0
            for v in jsn["voronois"][0][i]:
                (xx,yy) = genvoroxy(v)
                voronois[i][j].set_data(xx,yy)
                j+=1
    for i in range(jsn["number_of_robots"]):
        plans[i].set_data(jsn["planned_trajs"][0][i]["x"], jsn["planned_trajs"][0][i]["y"])
        originals[i].set_data(jsn["originals"][i]["x"], jsn["originals"][i]["y"])
    for i in range(jsn["number_of_robots"]):
        cpts = controlpoints[i]
        pts = jsn["controlpoints"][0][i]
        k = 0
        print(len(pts))
        while k < len(pts):
            cpts[k].set_data([pts[k][0]], [pts[k][1]])
            k+=1
    return to_animate


def animate(frame):
    global trajectories
    global jsn
    global to_animate
    global time_text
    global voronois
    global save
    global fig
    for i, atraj in enumerate(trajectories):
        x[i].append(jsn["points"][frame][i][0])
        y[i].append(jsn["points"][frame][i][1])
        atraj.set_data(x[i],y[i])
        circles[i].center = (jsn["points"][frame][i][0], jsn["points"][frame][i][1])

    if("voronois" in jsn and frame < len(jsn["voronois"]) and jsn["voronois"][frame] != None):
        for j in range(jsn["number_of_robots"]):
            for i,v in enumerate(jsn["voronois"][frame][j]):
                (xx,yy) = genvoroxy(v)
                voronois[j][i].set_data(xx, yy)

    if(frame < len(jsn["planned_trajs"]) and jsn["planned_trajs"][frame] != None):
        for i in range(jsn["number_of_robots"]):
            plans[i].set_data(jsn["planned_trajs"][frame][i]["x"], jsn["planned_trajs"][frame][i]["y"])

    for i in range(jsn["number_of_robots"]):
        originals[i].set_data(jsn["originals"][i]["x"][frame:], jsn["originals"][i]["y"][frame:])
    time_text.set_text('time = %.1f' % (jsn["dt"]*frame))

    if(frame < len(jsn["controlpoints"]) and jsn["controlpoints"][frame] != None):
        for i in range(jsn["number_of_robots"]):
            cpts = controlpoints[i]
            pts = jsn["controlpoints"][frame][i]
            k = 0
            print(len(pts))
            while k < len(pts):
                cpts[k].set_data([pts[k][0]], [pts[k][1]])
                k+=1

    if save:
        if(frame > 300):
            fig.savefig("images/"+str(frame)+".png")
    return to_animate

anim = animation.FuncAnimation(fig, animate, init_func=init,frames=len(jsn["points"]),interval=jsn["dt"]*1000,blit=True)

plt.show()

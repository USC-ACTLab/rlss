import json
import sys
from matplotlib import pyplot as plt
from matplotlib import animation
import os
import csv
from shapely.geometry.polygon import LinearRing, Polygon
from descartes import PolygonPatch

jsn = json.load(open(sys.argv[1]))
obstacles = os.listdir(sys.argv[2])


fig = plt.figure()
ax = plt.axes(xlim=(-10,10), ylim=(-10,10))
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
trajectories = []
to_animate = []

polygon_patches = []
for obs in obstacles:
    csvfile = open(sys.argv[2] + "/" + obs, "r")
    readr = csv.reader(csvfile, delimiter=',')
    points = []
    header = True
    for row in readr:
        if(header):
            header = False
            continue
        points.append((float(row[0]), float(row[1])))
    poly = Polygon(points)
    polypatch = PolygonPatch(poly, color='yellow', zorder=5)
    ax.add_patch(polypatch)
    to_animate.append(polypatch)

for i in range(jsn["number_of_robots"]):
    atraj = ax.plot([], [], lw=2, zorder=0)[0]
    trajectories.append(atraj)
    to_animate.append(atraj)


to_animate.append(time_text)
x = []
y = []

def init():
    global trajectories
    global to_animate
    global time_text
    global x
    global y
    x = []
    y = []
    for atraj in trajectories:
        atraj.set_data([],[])
        x.append([])
        y.append([])
    time_text.set_text('')
    return to_animate


def animate(frame):
    global trajectories
    global jsn
    global to_animate
    global time_text
    for i, atraj in enumerate(trajectories):
        x[i].append(jsn["points"][frame][i][0])
        y[i].append(jsn["points"][frame][i][1])
        atraj.set_data(x[i],y[i])
    time_text.set_text('time = %.1f' % (jsn["dt"]*frame))
    return to_animate

anim = animation.FuncAnimation(fig, animate, init_func=init,frames=len(jsn["points"]),interval=jsn["dt"]*1000,blit=True)

plt.show(block=False)

input()

import json
import argparse
from matplotlib import pyplot as plt
from matplotlib import animation
import os
import csv
from shapely.geometry.polygon import LinearRing, Polygon
from descartes import PolygonPatch
from matplotlib.patches import Rectangle
import numpy as np


def frameGen():
  global frameId
  global numFrames
  global args
  frameId += args.frameinc
  if frameId >= numFrames:
    frameId = 0
  yield frameId

def onKey(event):
  global anim_running
  global anim
  global step
  global args
  global frameId
  # print('you pressed', event.key, event.xdata, event.ydata)
  if event.key == ' ': # space => pause/continue
    if anim_running:
      anim.event_source.stop()
      anim_running = False
    else:
      anim.event_source.start()
      anim_running = True
      step = False
  if event.key == 'n':
    step = True
    anim.event_source.start()
    anim_running = True
  if event.key == 'p':
    step = True
    frameId -= 2 * args.frameinc
    anim.event_source.start()
    anim_running = True




def genvoroxy(voronoi):
    a = voronoi[0]
    b = voronoi[1]
    c = voronoi[2]
    xc = np.arange(-5,5,0.01).tolist()
    yc = []
    for xx in xc:
      if b != 0.0:
        yc.append((c-a*xx)/b)
      else:
        yc.append(0.0)
    return (xc,yc)

def init():
    global trajectories
    global to_animate
    global time_text
    global x
    global y
    global voronois
    global controlpoints

    if frameId != 0:
      return to_animate

    x = []
    y = []
    for atraj in trajectories:
        atraj.set_data([],[])
        x.append([])
        y.append([])
    time_text.set_text('')
    if args.voronoi and "voronois" in jsn:
        for i in range(jsn["number_of_robots"]):
            j = 0
            for v in jsn["voronois"][0][i]:
                (xx,yy) = genvoroxy(v)
                voronois[i][j].set_data(xx,yy)
                j+=1
    for i in range(jsn["number_of_robots"]):
        plans[i].set_data(jsn["planned_trajs"][0][i]["x"], jsn["planned_trajs"][0][i]["y"])
        originals[i].set_data(jsn["originals"][i]["x"], jsn["originals"][i]["y"])
    if args.controlpoints:
      for i in range(jsn["number_of_robots"]):
          cpts = controlpoints[i]
          pts = jsn["controlpoints"][0][i]
          k = 0
          # print(len(pts))
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
    global hyperplanes
    global args
    global fig
    global step
    global x
    global y

    for i, atraj in enumerate(trajectories):
        x[i].append(jsn["points"][frame][i][0])
        y[i].append(jsn["points"][frame][i][1])
        atraj.set_data(x[i],y[i])
        circles[i].center = (jsn["points"][frame][i][0], jsn["points"][frame][i][1])

    if args.voronoi and "voronois" in jsn and frame < len(jsn["voronois"]) and jsn["voronois"][frame] != None:
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

    if args.controlpoints and frame < len(jsn["controlpoints"]) and jsn["controlpoints"][frame] != None:
        for i in range(jsn["number_of_robots"]):
            cpts = controlpoints[i]
            if i < len(jsn["controlpoints"][frame]):
              pts = jsn["controlpoints"][frame][i]
              if pts:
                k = 0
                # print(len(pts))
                while k < len(pts):
                    cpts[k].set_data([pts[k][0]], [pts[k][1]])
                    k+=1

    if "discrete_plan" in jsn and frame < len(jsn["discrete_plan"]) and jsn["discrete_plan"][frame] != None:
        for i in range(jsn["number_of_robots"]):
          if i < len(jsn["discrete_plan"][frame]) and jsn["discrete_plan"][frame][i]:
            discrete_plans[i].set_data(jsn["discrete_plan"][frame][i]["x"], jsn["discrete_plan"][frame][i]["y"])
    else:
        for i in range(jsn["number_of_robots"]):
            discrete_plans[i].set_data([], [])


    hpIdx = 0
    if args.hyperplanes and "hyperplanes" in jsn and jsn["hyperplanes"][frame]:
      for hp in jsn["hyperplanes"][frame][0]:
        if hp:
          (hpx,hpy) = genvoroxy(hp)
          if hpIdx >= len(hyperplanes):
            v = ax.plot(hpx, hpy, lw=2, zorder=7, color=colors[hp[3]])[0] # color=colors[0])[0]
            hyperplanes.append(v)
            to_animate.append(v)
            # print(frame, hpIdx)
          else:
            hyperplanes[hpIdx].set_data(hpx, hpy)
          hpIdx += 1
    for idx in range(hpIdx, len(hyperplanes)):
      hyperplanes[idx].set_data([], [])

    if "occupied_cells" in jsn:
      cell_size = jsn["cell_size"]
      ocIdx = 0
      for xx,yy in zip(jsn["occupied_cells"][frame]["x"], jsn["occupied_cells"][frame]["y"]):
        # cell = ax.add_artist(Rectangle((xx-cell_size/2.0, yy-cell_size/2.0), cell_size, cell_size))
        occupied_cells[ocIdx].set_x(xx-cell_size/2.0)
        occupied_cells[ocIdx].set_y(yy-cell_size/2.0)
        ocIdx += 1

    if args.save:
        if(frame > 300):
            fig.savefig("images/"+str(frame)+".png")

    if step:
      anim.event_source.stop()
      anim_running = False

    return to_animate

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("file", help="input file containing data (*.json)")

  parser.add_argument('--save', dest='save', action='store_true')
  parser.add_argument('--no-save', dest='save', action='store_false')
  parser.set_defaults(save=False)

  parser.add_argument('--voronoi', dest='voronoi', action='store_true')
  parser.add_argument('--no-voronoi', dest='voronoi', action='store_false')
  parser.set_defaults(voronoi=True)

  parser.add_argument('--controlpoints', dest='controlpoints', action='store_true')
  parser.add_argument('--no-controlpoints', dest='controlpoints', action='store_false')
  parser.set_defaults(controlpoints=True)

  parser.add_argument('--frameinc', default=1, type=int, help="visualize every frameinc frame")

  parser.add_argument('--blit', dest='blit', action='store_true')
  parser.add_argument('--no-blit', dest='blit', action='store_false')
  parser.set_defaults(blit=True)

  parser.add_argument('--hyperplanes', dest='hyperplanes', action='store_true')
  parser.add_argument('--no-hyperplanes', dest='hyperplanes', action='store_false')
  parser.set_defaults(hyperplanes=True)

  args = parser.parse_args()

  jsn = json.load(open(args.file))

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

  occupied_cells = []

  if "occupied_cells" in jsn:
    cell_size = jsn["cell_size"]
    for x,y in zip(jsn["occupied_cells"][0]["x"], jsn["occupied_cells"][0]["y"]):
      cell = ax.add_artist(Rectangle((x-cell_size/2.0, y-cell_size/2.0), cell_size, cell_size, alpha=0.5))
      occupied_cells.append(cell)
      to_animate.append(cell)
    # cells = ax.scatter(jsn["occupied_cells"]["x"], jsn["occupied_cells"]["y"], marker='s')
    # to_animate.append(cells)



  circles = []
  voronois = []
  plans = []
  originals = []
  controlpoints = []
  discrete_plans = []
  hyperplanes = []

  for i in range(jsn["number_of_robots"]):
      orig = ax.plot(jsn["originals"][i]["x"], jsn["originals"][i]["y"], linestyle="dashed", lw=2, zorder = 10, color=colors[i], alpha = 0.8)[0]
      originals.append(orig)
      to_animate.append(orig)

  for i in range(jsn["number_of_robots"]):
      x = jsn["planned_trajs"][0][i]["x"]
      y = jsn["planned_trajs"][0][i]["y"]
      tr = ax.plot(x, y, lw=2, zorder = 10, color=colors[i], alpha=0.4)[0]
      plans.append(tr)
      to_animate.append(tr)

  if args.controlpoints:
    for i in range(jsn["number_of_robots"]):
        pts = jsn["controlpoints"][0][i]
        cpts = []
        # print(len(pts))
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

  for i in range(jsn["number_of_robots"]):
      discrete_plan = ax.plot([], [], linestyle=":", lw=2, zorder = 10, color=colors[i], alpha = 0.8)[0]
      discrete_plans.append(discrete_plan)
      to_animate.append(discrete_plan)


  if args.voronoi and "voronois" in jsn:
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

  anim_running = True
  step = False
  frameId = 0
  numFrames = len(jsn["points"])
  # fig.canvas.mpl_connect('button_press_event', onClick)
  fig.canvas.mpl_connect('key_press_event', onKey)
  anim = animation.FuncAnimation(fig, animate, init_func=init,frames=frameGen,interval=jsn["dt"]*1000,blit=args.blit,repeat=True)

  plt.show()





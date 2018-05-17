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
  if event.key == 'h':
    args.hyperplanes = not args.hyperplanes
    if not anim_running:
      step = True
      frameId -= args.frameinc
      anim.event_source.start()
      anim_running = True

  if event.key == 'v':
    args.voronoi = not args.voronoi
    if not anim_running:
      step = True
      frameId -= args.frameinc
      anim.event_source.start()
      anim_running = True


def genvoroxy(voronoi):
    a = voronoi[0]
    b = voronoi[1]
    c = voronoi[2]
    #xc = np.arange(-5,5,0.01).tolist()
    xc = [-5,5]
    yc = []
    for xx in xc:
      if b != 0.0:
        yc.append((c-a*xx)/b)
      else:
        yc.append(0.0)
    return (xc,yc)

def genvoronormal(voronoi):
    a = voronoi[0]
    b = voronoi[1]
    c = voronoi[2]
    xstart = a*c
    ystart = b*c
    xend = a*(c+0.1)
    yend = b*(c+0.1)
    return ([xstart,xend], [ystart,yend])

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

    x = dict()
    y = dict()
    for i in robots:
        trajectories[i].set_data([],[])
        x[i] = []
        y[i] = []
    time_text.set_text('')
    if args.voronoi and "voronois" in jsn:
        for i in robots:
            j = 0
            for v in jsn["voronois"][0][i]:
                (xx,yy) = genvoroxy(v)
                voronois[i][j].set_data(xx,yy)
                (xx,yy) = genvoronormal(v)
                voronoinormals[i][j].set_data(xx,yy)
                j+=1
    if "planned_trajs" in jsn:
      for i in robots:
          plans[i].set_data(jsn["planned_trajs"][0][i]["x"], jsn["planned_trajs"][0][i]["y"])
          originals[i].set_data(jsn["originals"][i]["x"], jsn["originals"][i]["y"])
    if args.controlpoints:
      for i in robots:
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

    for i in robots:
        x[i].append(jsn["points"][frame][i][0])
        y[i].append(jsn["points"][frame][i][1])
        trajectories[i].set_data(x[i],y[i])
        circles[i].center = (jsn["points"][frame][i][0], jsn["points"][frame][i][1])

    if args.voronoi and "voronois" in jsn and frame < len(jsn["voronois"]) and jsn["voronois"][frame] != None:
        for j in robots:
            for i,v in enumerate(jsn["voronois"][frame][j]):
                (xx,yy) = genvoroxy(v)
                voronois[j][i].set_data(xx, yy)
                (xx,yy) = genvoronormal(v)
                voronoinormals[j][i].set_data(xx,yy)

    if "planned_trajs" in jsn and frame < len(jsn["planned_trajs"]) and jsn["planned_trajs"][frame] != None:
        for i in robots:
            plans[i].set_data(jsn["planned_trajs"][frame][i]["x"], jsn["planned_trajs"][frame][i]["y"])

    for i in robots:
        originals[i].set_data(jsn["originals"][i]["x"][frame:], jsn["originals"][i]["y"][frame:])
    time_text.set_text('time = %.1f' % (jsn["dt"]*frame))

    if args.controlpoints and frame < len(jsn["controlpoints"]) and jsn["controlpoints"][frame] != None:
        for i in robots:
            cpts = controlpoints[i]
            if i < len(jsn["controlpoints"][frame]):
              pts = jsn["controlpoints"][frame][i]
              if pts:
                k = 0
                # print(len(pts))
                while k < len(pts):
                    cpts[k].set_data([pts[k][0]], [pts[k][1]])
                    k+=1

    if args.controlpoints_guessed and frame < len(jsn["controlpoints_guessed"]) and jsn["controlpoints_guessed"][frame] != None:
        for i in robots:
            cpts = controlpoints_guessed[i]
            if i < len(jsn["controlpoints_guessed"][frame]):
              pts = jsn["controlpoints_guessed"][frame][i]
              if pts:
                k = 0
                # print(len(pts))
                while k < len(pts):
                    cpts[k].set_data([pts[k][0]], [pts[k][1]])
                    k+=1

    if "discrete_plan" in jsn and frame < len(jsn["discrete_plan"]) and jsn["discrete_plan"][frame] != None:
        for i in robots:
          if i < len(jsn["discrete_plan"][frame]) and jsn["discrete_plan"][frame][i]:
            discrete_plans[i].set_data(jsn["discrete_plan"][frame][i]["x"], jsn["discrete_plan"][frame][i]["y"])
    elif frame % 10 == 0:
        for i in robots:
            discrete_plans[i].set_data([], [])


    hpIdx = 0
    robot = 0
    if args.hyperplanes and "hyperplanes" in jsn and frame < len(jsn["hyperplanes"]) and jsn["hyperplanes"][frame] and robot < len(jsn["hyperplanes"][frame]) and jsn["hyperplanes"][frame][robot]:
      for hp in jsn["hyperplanes"][frame][robot]:
        if hp: # and hp[3] == 0:
          (hpx,hpy) = genvoroxy(hp)
          (hpnx, hpny) = genvoronormal(hp)
          if hpIdx >= len(hyperplanes):
            v = ax.plot(hpx, hpy, lw=2, zorder=7, color=colors[hp[3]])[0] # color=colors[0])[0]
            hyperplanes.append(v)
            to_animate.append(v)
            v = ax.plot(hpnx, hpny, lw=2, zorder=7, color=colors[hp[3]])[0] # color=colors[0])[0]
            hyperplanenormals.append(v)
            to_animate.append(v)

            # print(frame, hpIdx)
          else:
            hyperplanes[hpIdx].set_data(hpx, hpy)
            hyperplanes[hpIdx].set_color(colors[hp[3]])
            hyperplanenormals[hpIdx].set_data(hpnx, hpny)
            hyperplanenormals[hpIdx].set_color(colors[hp[3]])
          hpIdx += 1
    for idx in range(hpIdx, len(hyperplanes)):
      hyperplanes[idx].set_data([], [])
      hyperplanenormals[idx].set_data([], [])

    # if "occupied_cells" in jsn:
    #   cell_size = jsn["cell_size"]
    #   ocIdx = 0
    #   for xx,yy in zip(jsn["occupied_cells"][frame]["x"], jsn["occupied_cells"][frame]["y"]):
    #     # cell = ax.add_artist(Rectangle((xx-cell_size/2.0, yy-cell_size/2.0), cell_size, cell_size))
    #     occupied_cells[ocIdx].set_x(xx-cell_size/2.0)
    #     occupied_cells[ocIdx].set_y(yy-cell_size/2.0)
    #     ocIdx += 1

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

  parser.add_argument('--controlpoints-guessed', dest='controlpoints_guessed', action='store_true')
  parser.add_argument('--no-controlpoints-guessed', dest='controlpoints_guessed', action='store_false')
  parser.set_defaults(controlpoints_guessed=True)

  parser.add_argument('--frameinc', default=1, type=int, help="visualize every frameinc frame")

  parser.add_argument('--blit', dest='blit', action='store_true')
  parser.add_argument('--no-blit', dest='blit', action='store_false')
  parser.set_defaults(blit=True)

  parser.add_argument('--hyperplanes', dest='hyperplanes', action='store_true')
  parser.add_argument('--no-hyperplanes', dest='hyperplanes', action='store_false')
  parser.set_defaults(hyperplanes=True)

  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")

  args = parser.parse_args()

  jsn = json.load(open(args.file))

  colors = ['b', 'g', 'r', 'c', 'm', 'k'] * 10

  # fig = plt.figure()
  # ax = plt.axes(xlim=(-4,4), ylim=(-4,4))
  # ax.set_aspect('equal')
  xmin = -2.5
  xmax = 2.5
  ymin = -1.0
  ymax = 2.5

  aspect = (xmax - xmin) / (ymax - ymin)

  if args.video:
    fig = plt.figure(frameon=False, figsize=(6 * aspect, 6))
    ax = fig.add_subplot(111, aspect='equal')
    fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
  else:
    fig = plt.figure()
    ax = plt.axes(xlim=(-4,4), ylim=(-4,4))
    ax.set_aspect('equal')

  plt.xlim(xmin, xmax)
  plt.ylim(ymin, ymax)

  # plt.axis('off')
  time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
  trajectories = dict()
  to_animate = []

  polygon_patches = []
  if("obstacles" in jsn):
      for obs in jsn["obstacles"]:
          poly = Polygon(obs)
          polypatch = PolygonPatch(poly, color='gray', zorder = 5)
          ax.add_patch(polypatch)
          to_animate.append(polypatch)

  occupied_cells = []

  if "occupied_cells" in jsn:
    cell_size = jsn["cell_size"]
    for x,y in zip(jsn["occupied_cells"]["x"], jsn["occupied_cells"]["y"]):
      cell = ax.add_artist(Rectangle((x-cell_size/2.0, y-cell_size/2.0), cell_size, cell_size, alpha=0.5))
      occupied_cells.append(cell)
      to_animate.append(cell)
    # cells = ax.scatter(jsn["occupied_cells"]["x"], jsn["occupied_cells"]["y"], marker='s')
    # to_animate.append(cells)



  circles = dict()
  voronois = dict()
  voronoinormals = dict()
  plans = dict()
  originals = dict()
  controlpoints = dict()
  controlpoints_guessed = dict()
  discrete_plans = dict()
  hyperplanes = []
  hyperplanenormals = []
  robots = range(jsn["number_of_robots"]) #[0,1]

  for i in robots:
      orig = ax.plot(jsn["originals"][i]["x"], jsn["originals"][i]["y"], linestyle="dashed", lw=2, zorder = 10, color=colors[i], alpha = 0.8)[0]
      originals[i] = orig #.append(orig)
      to_animate.append(orig)

  if "planned_trajs" in jsn:
    for i in robots:
        x = jsn["planned_trajs"][0][i]["x"]
        y = jsn["planned_trajs"][0][i]["y"]
        tr = ax.plot(x, y, lw=2, zorder = 10, color=colors[i], alpha=0.4)[0]
        plans[i] = tr #.append(tr)
        to_animate.append(tr)

  if args.controlpoints:
    for i in robots:
        pts = jsn["controlpoints"][0][i]
        cpts = []
        # print(len(pts))
        for idx, pt in enumerate(pts):
            ptd = ax.plot([pt[0]], [pt[1]], zorder = 20, color=colors[int(idx/8.0)], alpha = 0.8, marker='o')[0]
            cpts.append(ptd)
            to_animate.append(ptd)
        controlpoints[i] = cpts

  if args.controlpoints_guessed:
    for i in robots:
        pts = jsn["controlpoints_guessed"][0][i]
        cpts = []
        # print(len(pts))
        for idx, pt in enumerate(pts):
            ptd = ax.plot([pt[0]], [pt[1]], zorder = 20, color=colors[int(idx/8.0)], alpha = 0.8, marker='o')[0]
            cpts.append(ptd)
            to_animate.append(ptd)
        controlpoints_guessed[i] = cpts

  for i in robots:
      atraj = ax.plot([], [], lw=2, zorder=0, color = colors[i])[0]
      trajectories[i] = atraj #.append(atraj)
      to_animate.append(atraj)

      circle = plt.Circle((jsn["points"][0][i][0],jsn["points"][0][i][1]), radius=jsn["robot_radius"], fc=colors[i], lw=0, alpha=0.2)
      circles[i] = circle #.append(circle)
      to_animate.append(circle)
      ax.add_artist(circle)

  for i in robots:
      discrete_plan = ax.plot([], [], linestyle=":", lw=2, zorder = 10, color=colors[i], alpha = 0.8)[0]
      discrete_plans[i] = discrete_plan
      to_animate.append(discrete_plan)


  if args.voronoi and "voronois" in jsn:
      for i in robots:
          voronois[i] = []
          voronoinormals[i] = []
          # voronois.append([])
          for v in jsn["voronois"][0][i]:
              (x,y) = genvoroxy(v)
              (x2,y2) = genvoronormal(v)
              v = ax.plot(x, y, lw=2, zorder=7, color=colors[i])[0]
              voronois[i].append(v)
              to_animate.append(v)
              v = ax.plot(x2, y2, lw=2, zorder=7, color=colors[i])[0]
              voronoinormals[i].append(v)
              to_animate.append(v)

  to_animate.append(time_text)
  x = []
  y = []

  anim_running = True
  step = False
  frameId = 0
  numFrames = len(jsn["points"])

  if args.video:
    SPEED = 1
    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=range(0, numFrames, args.frameinc))
    anim.save(
      args.video,
      "ffmpeg",
      fps=1.0 / jsn["dt"] / args.frameinc * SPEED,
      dpi=150)
  else:
    fig.canvas.mpl_connect('key_press_event', onKey)
    anim = animation.FuncAnimation(fig, animate, init_func=init,frames=frameGen, interval=jsn["dt"]*1000,blit=args.blit,repeat=True)
    plt.show()

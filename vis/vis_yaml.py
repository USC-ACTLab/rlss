import yaml
import argparse
from matplotlib import pyplot as plt
from matplotlib import animation
import os
import csv
# from shapely.geometry.polygon import LinearRing, Polygon
# from descartes import PolygonPatch
from matplotlib.patches import Rectangle
import numpy as np

def genvoroxy(normal, dist):
    a = normal[0]
    b = normal[1]
    c = dist
    #xc = np.arange(-5,5,0.01).tolist()
    xc = [-5,5]
    yc = []
    for xx in xc:
      if b != 0.0:
        yc.append((c-a*xx)/b)
      else:
        yc.append(0.0)
    return (xc,yc)

def frameGen():
  global frameId
  global numFrames
  global args
  frameId += 1 #args.frameinc
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

def animate(frame):
  global anim_running
  global step

  it = data["iterations"][frame]

  time_text.set_text('time = %.1f' % it["time"])

  for i, pos in enumerate(it["positions"]):
    positions[i].center = pos

  for idx, pt in enumerate(it["controlpointsGuessed"]):
    controlpointsGuessed[idx].set_data([pt[0]], [pt[1]])

  for idx, pt in enumerate(it["controlpoints"]):
    controlpoints[idx].set_data([pt[0]], [pt[1]])

  # for idx, hp in enumerate(it["hyperplanes"]):
  #   (x,y) = genvoroxy(hp["normal"], hp["dist"])
  #   hyperplanes[idx].set_data(x, y)
  #   hyperplanes[idx].set_color(colors[hp["piece"]])

  if step:
    anim.event_source.stop()
    anim_running = False

  return to_animate

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("file", help="input file containing data (*.yaml)")

  args = parser.parse_args()

  data = yaml.load(open(args.file))

  colors = ['b', 'g', 'r', 'c', 'm', 'k'] * 10
  positions = []
  controlpointsGuessed = []
  controlpoints = []
  hyperplanes = []
  to_animate = []


  fig = plt.figure()
  ax = plt.axes(xlim=(-4,4), ylim=(-4,4))
  ax.set_aspect('equal')

  time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
  to_animate.append(time_text)

  env = data["environment"]
  cell_size = env["cellSize"]
  for obs in env["obstacles"]:
    ax.add_artist(Rectangle((obs[0]-cell_size/2.0, obs[1]-cell_size/2.0), cell_size, cell_size, alpha=0.5))



  i = 0
  it = data["iterations"][i]

  for i, pos in enumerate(it["positions"]):
    circle = plt.Circle(pos, radius=env["robot_radius"], fc=colors[i], lw=0, alpha=0.2)
    ax.add_artist(circle)
    positions.append(circle)
    to_animate.append(circle)

  for idx, pt in enumerate(it["controlpointsGuessed"]):
    cpt = ax.plot([pt[0]], [pt[1]], zorder = 20, color=colors[int(idx/8.0)], alpha = 0.2, marker='o')[0]
    controlpointsGuessed.append(cpt)
    to_animate.append(cpt)

  for idx, pt in enumerate(it["controlpoints"]):
    cpt = ax.plot([pt[0]], [pt[1]], zorder = 20, color=colors[int(idx/8.0)], alpha = 0.8, marker='o')[0]
    controlpoints.append(cpt)
    to_animate.append(cpt)

  for idx, hp in enumerate(it["hyperplanes"]):
    (x,y) = genvoroxy(hp["normal"], hp["dist"])
    v = ax.plot(x, y, lw=2, zorder=7, color=colors[hp["piece"]])[0]
    hyperplanes.append(v)
    to_animate.append(v)

  anim_running = True
  step = False
  frameId = 0
  numFrames = len(data["iterations"])
  fig.canvas.mpl_connect('key_press_event', onKey)
  anim = animation.FuncAnimation(fig, animate, frames=frameGen, interval=100, blit=False, repeat=True)

  plt.show()




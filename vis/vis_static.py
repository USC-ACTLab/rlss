import json
import argparse
from matplotlib import pyplot as plt
from shapely.geometry.polygon import LinearRing, Polygon
from descartes import PolygonPatch
from matplotlib.patches import Rectangle
import matplotlib.gridspec as gridspec
import numpy as np


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("file", help="input file containing data (*.json)")
  parser.add_argument('--output', default=None, help="output file (show on screen if unspecified)")

  args = parser.parse_args()

  jsn = json.load(open(args.file))
  numFrames = len(jsn["points"])
  dt = jsn["dt"]
  t = np.arange(0, numFrames * dt, dt)

  colors = ['b', 'g', 'r', 'c', 'm', 'k'] * 10

  gs = gridspec.GridSpec(5, 1)
  fig = plt.figure()

  # plot trajectories
  ax = plt.subplot(gs[0:3, 0])
  ax.set_aspect('equal')

  polygon_patches = []
  if("obstacles" in jsn):
    for obs in jsn["obstacles"]:
      poly = Polygon(obs)
      polypatch = PolygonPatch(poly, color='gray', zorder = 5)
      ax.add_patch(polypatch)

  for i in range(jsn["number_of_robots"]):
    ax.plot(jsn["originals"][i]["x"], jsn["originals"][i]["y"], linestyle="dashed", lw=2, zorder = 10, color=colors[i], alpha = 0.8)

  for i in range(jsn["number_of_robots"]):
    x = []
    y = []
    for frame in range(len(jsn["points"])):
      x.append(jsn["points"][frame][i][0])
      y.append(jsn["points"][frame][i][1])
    ax.plot(x, y, lw=2, zorder=0, color = colors[i])

  # plot velocity
  ax = plt.subplot(gs[3, 0]) # row 2

  for i in range(jsn["number_of_robots"]):
    vel = np.empty((len(t), 2))
    for frame in range(len(jsn["points"])):
      vel[frame, 0:2] = [jsn["velocities"][frame][i][0], jsn["velocities"][frame][i][1]]
    ax.plot(t, np.linalg.norm(vel[:,0:2], axis=1), lw=2, zorder=0, color = colors[i])
  ax.set_xlabel("time [s]")
  ax.set_ylabel("velocity [m/s]")

  # plot acceleration
  ax = plt.subplot(gs[4, 0]) # row 2

  for i in range(jsn["number_of_robots"]):
    acc = np.empty((len(t), 2))
    for frame in range(len(jsn["points"])):
      acc[frame, 0:2] = [jsn["accelerations"][frame][i][0], jsn["accelerations"][frame][i][1]]
    ax.plot(t, np.linalg.norm(acc[:,0:2], axis=1), lw=2, zorder=0, color = colors[i])
  ax.set_xlabel("time [s]")
  ax.set_ylabel("acceleration [m/s^2]")

  if args.output:
    fig.savefig(args.output)
  else:
    plt.show()





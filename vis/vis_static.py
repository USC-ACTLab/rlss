import json
import argparse
from matplotlib import pyplot as plt
from shapely.geometry.polygon import LinearRing, Polygon
from descartes import PolygonPatch
from matplotlib.patches import Rectangle
import numpy as np


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("file", help="input file containing data (*.json)")
  parser.add_argument('--output', default=None, help="output file (show on screen if unspecified)")

  args = parser.parse_args()

  jsn = json.load(open(args.file))

  colors = ['b', 'g', 'r', 'c', 'm', 'k']

  fig = plt.figure()
  ax = plt.axes()
  ax.set_aspect('equal')

  polygon_patches = []
  if("obstacles" in jsn):
    for obs in jsn["obstacles"]:
      poly = Polygon(obs)
      polypatch = PolygonPatch(poly, color='yellow', zorder = 5)
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

  if args.output:
    fig.savefig(args.output)
  else:
    plt.show()





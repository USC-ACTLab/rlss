import numpy as np
import argparse

import matplotlib.pyplot as plt
from matplotlib import colors

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("csvfile", help="input file containing statistics (*.csv)")
  parser.add_argument("--robot", default='all', help="robot number for details for one robot, or all for overview of all robots")
  args = parser.parse_args()

  colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
  entries = ["total", "Astar", "SVM", "QP"]

  csvfile = args.csvfile

  data = np.loadtxt(csvfile, delimiter=',', skiprows=1)

  dt = data[1,0] - data[0,0]
  num_robots = int((data.shape[1] - 1) / 4)

  if args.robot == 'all':
    # width = dt / num_robots
    for robot in range(num_robots):
      column = 1 + 4 * robot
      # plt.bar(data[:,0] + (column-1) * width, data[:,column], width=width, color=colors[column-1])
      plt.plot(data[:,0], data[:,column], color=colors[robot], label=robot)
  else:
    robot = int(args.robot)
    # for c in range(len(entries)):
    #   column = 1 + 4 * robot + c
    #   plt.plot(data[:,0], data[:,column], label=entries[c])
    # print(data[:,0].shape)
    # print(data[:,1+4*robot+1:1+4*robot+4].shape)
    plt.plot(data[:,0], data[:,1+4*robot], label=entries[0])
    plt.stackplot(data[:,0], data[:,1+4*robot+1:1+4*robot+4].T, labels=entries[1:])

  plt.xlabel("Time [s]")
  plt.ylabel("Optimization time [ms]")
  plt.legend()
  plt.show()

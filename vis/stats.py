import numpy as np
import argparse

import matplotlib.pyplot as plt
from matplotlib import colors

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("csvfile", help="input file containing statistics (*.csv)")
  args = parser.parse_args()

  colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

  csvfile = args.csvfile

  data = np.loadtxt(csvfile, delimiter=',', skiprows=1)

  dt = data[1,0] - data[0,0]
  num_robots = data.shape[1] - 1
  width = dt / num_robots
  for column in range(1, data.shape[1]):
    # plt.bar(data[:,0] + (column-1) * width, data[:,column], width=width, color=colors[column-1])
    plt.plot(data[:,0] + (column-1) * width, data[:,column], color=colors[column-1])
  plt.xlabel("Time [s]")
  plt.ylabel("Optimization time [ms]")
  plt.show()

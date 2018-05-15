import numpy as np
import argparse

import matplotlib.pyplot as plt
from matplotlib import colors

import json

width = 0.35

parser = argparse.ArgumentParser(description="compare stats")
parser.add_argument('--rvo2', dest='rvo2')
parser.add_argument('--rme', dest='rme')

args = parser.parse_args()

rvo2 = json.load(open(args.rvo2))
rme = json.load(open(args.rme))

if(rvo2["number_of_robots"] != rme["number_of_robots"]):
    exit("Different number of robots")







fig = plt.figure()
ind = np.arange(rvo2["number_of_robots"])


avg_cycles_rme = list(rme["average_cycle_times"])
avg_cycles_rvo2 = list(rvo2["average_cycle_times"])
ax = fig.add_subplot(231)


rects1 = ax.bar(ind, avg_cycles_rme, width, color="royalblue")
rects2 = ax.bar(ind+width, avg_cycles_rvo2, width, color="black")


ax.set_ylabel('avg. cycle time')
ax.set_xlabel('robot id')
ax.set_title('Average Cycle Times')

ax.set_xticks(ind+width/2)
tpl = ()
for i in range(1, rvo2["number_of_robots"]+1):
    tpl += (i,)

ax.set_xticklabels(tpl)
ax.legend((rects1[0], rects2[0]), ("rme", "rvo2"))


maximum_cycles_rme = list(rme["maximum_cycle_times"])
maximum_cycles_rvo2 = list(rvo2["maximum_cycle_times"])
ax2 = fig.add_subplot(232)
rects3 = ax2.bar(ind, maximum_cycles_rme, width, color="royalblue")
rects4 = ax2.bar(ind+width, maximum_cycles_rvo2, width, color="black")


ax2.set_ylabel('max. cycle time')
ax2.set_xlabel('robot id')
ax2.set_title('Maximum Cycle Times')

ax2.set_xticks(ind+width/2)
tpl = ()
for i in range(1, rvo2["number_of_robots"]+1):
    tpl += (i,)

ax2.set_xticklabels(tpl)
ax2.legend((rects3[0], rects4[0]), ("rme", "rvo2"))




robot_no_crash_percentages_rvo2 = []
obstacle_no_crash_percentages_rvo2 = []
total_no_crash_percentages_rvo2 = []
robot_no_crash_percentages_rme = []
obstacle_no_crash_percentages_rme = []
total_no_crash_percentages_rme = []

i = 0
while i < rvo2["number_of_robots"]:
    robot_no_crash_percentages_rvo2.append(100.0 * rvo2["robot_no_crash_counts"][i]/(rvo2["robot_no_crash_counts"][i] + rvo2["robot_crash_counts"][i]))
    robot_no_crash_percentages_rme.append(100.0 * rme["robot_no_crash_counts"][i]/(rme["robot_no_crash_counts"][i] + rme["robot_crash_counts"][i]))
    obstacle_no_crash_percentages_rvo2.append(100.0 * rvo2["obstacle_no_crash_counts"][i]/(rvo2["obstacle_no_crash_counts"][i] + rvo2["obstacle_crash_counts"][i]))
    obstacle_no_crash_percentages_rme.append(100.0 * rme["obstacle_no_crash_counts"][i]/(rme["obstacle_no_crash_counts"][i] + rme["obstacle_crash_counts"][i]))
    total_no_crash_percentages_rvo2.append(100.0 * (rvo2["robot_no_crash_counts"][i] + rvo2["obstacle_no_crash_counts"][i])/(rvo2["obstacle_no_crash_counts"][i] + rvo2["obstacle_crash_counts"][i]+ rvo2["robot_no_crash_counts"][i] + rvo2["robot_crash_counts"][i]))
    total_no_crash_percentages_rme.append(100.0 * (rme["robot_no_crash_counts"][i] + rme["obstacle_no_crash_counts"][i])/(rme["obstacle_no_crash_counts"][i] + rme["obstacle_crash_counts"][i] + rme["robot_no_crash_counts"][i] + rme["robot_crash_counts"][i]))
    i+=1

ax3 = fig.add_subplot(233)
rects5 = ax3.bar(ind, total_no_crash_percentages_rme, width, color="royalblue")
rects6 = ax3.bar(ind+width, total_no_crash_percentages_rvo2, width, color="black")


ax3.set_ylabel('total valid configuration percentage')
ax3.set_xlabel('robot id')
ax3.set_title('Total Valid Configurations')

ax3.set_xticks(ind+width/2)
tpl = ()
for i in range(1, rvo2["number_of_robots"]+1):
    tpl += (i,)

ax3.set_xticklabels(tpl)
ax3.legend((rects5[0], rects6[0]), ("rme", "rvo2"))



ax4 = fig.add_subplot(234)
rects7 = ax4.bar(ind, obstacle_no_crash_percentages_rme, width, color="royalblue")
rects8 = ax4.bar(ind+width, obstacle_no_crash_percentages_rvo2, width, color="black")


ax4.set_ylabel('total valid configuration percentage wrt obstacles')
ax4.set_xlabel('robot id')
ax4.set_title('Total Valid Configurations wrt Obstacles')

ax4.set_xticks(ind+width/2)
tpl = ()
for i in range(1, rvo2["number_of_robots"]+1):
    tpl += (i,)

ax4.set_xticklabels(tpl)
ax4.legend((rects7[0], rects8[0]), ("rme", "rvo2"))


ax5 = fig.add_subplot(235)
rects9 = ax5.bar(ind, robot_no_crash_percentages_rme, width, color="royalblue")
rects10 = ax5.bar(ind+width, robot_no_crash_percentages_rvo2, width, color="black")


ax5.set_ylabel('total valid configuration percentage wrt robots')
ax5.set_xlabel('robot id')
ax5.set_title('Total Valid Configurations wrt Robots')

ax5.set_xticks(ind+width/2)
tpl = ()
for i in range(1, rvo2["number_of_robots"]+1):
    tpl += (i,)

ax5.set_xticklabels(tpl)
ax5.legend((rects9[0], rects10[0]), ("rme", "rvo2"))

robots_reached_rme = 0
robots_reached_rvo2 = 0
for i in rvo2["robots_reached"]:
    if i:
        robots_reached_rvo2+=1

for i in rme["robots_reached"]:
    if i:
        robots_reached_rme+=1

ax6 = fig.add_subplot(236)
rects11 = ax6.bar(np.arange(1), [robots_reached_rme], width, color="royalblue")
rects12 = ax6.bar(np.arange(1) + width, [robots_reached_rvo2], width, color="black")


ax6.set_ylabel('number of robots reached to destination on time')
ax6.set_xlabel('robot id')
ax6.set_title('Robots Reached')

ax6.set_xticks(np.arange(1) + width)
tpl = ()
for i in range(1, rvo2["number_of_robots"]+1):
    tpl += (i,)

ax6.set_xticklabels(tpl)
ax6.legend((rects11[0], rects12[0]), ("rme", "rvo2"))
ax6.set_ylim(top=rvo2["number_of_robots"])


plt.show()

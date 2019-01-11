# Gets in put as follows
# a line for each obstacle box
# a line for each robot box
# a line for nth hyperplane seperating nth obstacle from all robot boxes
# hyperplane: ax + by + cz + d = 0
#
#
# o mx my mz Mx My Mz
# o mx my mz Mx My Mz
# r mx my myz Mx My Mz
# h a b c d
# h a b c d
#

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import time
import copy

log_file = open(sys.argv[1], "r")
lines = log_file.readlines()
log_file.close()

obstacle_boxes = []
robot_boxes = []
hyperplanes = []

for line in lines:
    if line.strip() == "":
        continue
    splitted = line.split(" ")
    if(splitted[0] == "o"):
        # obstacle box
        obstacle_boxes.append([float(splitted[1]), \
        float(splitted[2]), float(splitted[3]), float(splitted[4]), \
        float(splitted[5]), float(splitted[6])])
    elif(splitted[0] == "r"):
        # robot box
        robot_boxes.append([float(splitted[1]), \
        float(splitted[2]), float(splitted[3]), float(splitted[4]), \
        float(splitted[5]), float(splitted[6])])
    elif(splitted[0] == "h"):
        # hyperplane
        hyperplanes.append([float(splitted[1]), \
        float(splitted[2]), float(splitted[3]), float(splitted[4])])

def generatePoly3D(box, facecolors = 'w', linewidths = 1, alpha = 0.5):
    # convert box given as min and max points to
    # Poly3DCollection to be drawn
    min_x = box[0]
    min_y = box[1]
    min_z = box[2]
    max_x = box[3]
    max_y = box[4]
    max_z = box[5]
    v = [
    [min_x, min_y, min_z],
    [max_x, min_y, min_z],
    [max_x, min_y, max_z],
    [min_x, min_y, max_z],
    [min_x, max_y, min_z],
    [max_x, max_y, min_z],
    [max_x, max_y, max_z],
    [min_x, max_y, max_z]
    ]

    surfaces = [
        [v[0], v[1], v[2], v[3]],
        [v[3], v[7], v[6], v[2]],
        [v[7], v[4], v[5], v[6]],
        [v[4], v[5], v[1], v[6]],
        [v[7], v[4], v[0], v[3]],
        [v[6], v[5], v[1], v[2]]
    ]

    collection = Poly3DCollection(surfaces, linewidths = linewidths, alpha = alpha);
    collection.set_facecolor(facecolors)

    return collection


ROBOT_POLIES = []
for b in robot_boxes:
    ROBOT_POLIES.append(generatePoly3D(b, facecolors = 'b'))



for idx, box in enumerate(obstacle_boxes):
    fig = plt.figure(idx)
    ax = fig.add_subplot(111, projection='3d')

    robot_polies = copy.deepcopy(ROBOT_POLIES)

    for p in robot_polies:
        ax.add_collection3d(p)


    obs_poly = generatePoly3D(box, facecolors = 'r')
    ax.add_collection3d(obs_poly)

    # ax + by + cz + d = 0
    hp = hyperplanes[idx]
    a = hp[0]
    b = hp[1]
    c = hp[2]
    dist = hp[3]
    p1 = [-1 * a * dist, -1 * b * dist, -1 * c * dist]
    support = [1, 0, 0]
    dot = abs(support[0] * a + support[1] * b + support[2] * c)
    print(dot)
    if(dot > 0.999999):
        support = [0, 1, 0]

    d = support[0]
    e = support[1]
    f = support[2]

    pts = []


    mult = 10;
    v = [b*f-c*e, c*d-a*f,a*e-b*d]
    pts.append([p1[0] + v[0] * mult, p1[1] + v[1] * mult, p1[2] + v[2] * mult])
    d = v[0]
    e = v[1]
    f = v[2]
    v = [b*f-c*e, c*d-a*f,a*e-b*d]
    pts.append([p1[0] + v[0] * mult, p1[1] + v[1] * mult, p1[2] + v[2] * mult])
    d = v[0]
    e = v[1]
    f = v[2]
    v = [b*f-c*e, c*d-a*f,a*e-b*d]
    pts.append([p1[0] + v[0] * mult, p1[1] + v[1] * mult, p1[2] + v[2] * mult])
    d = v[0]
    e = v[1]
    f = v[2]
    v = [b*f-c*e, c*d-a*f,a*e-b*d]
    pts.append([p1[0] + v[0] * mult, p1[1] + v[1] * mult, p1[2] + v[2] * mult])

    surfaces = [[pts[0], pts[1], pts[2], pts[3]]]

    print(surfaces)

    plane = Poly3DCollection(surfaces, linewidths = 1, alpha = 0.5)
    plane.set_facecolor('g')

    ax.add_collection3d(plane)
    ax.plot([p1[0], p1[0] + a], [p1[1], p1[1] + b], [p1[2], p1[2] + c])


plt.show()

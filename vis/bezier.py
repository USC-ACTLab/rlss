import csv
import sys

import matplotlib.pyplot as plt
from matplotlib import colors
import six
colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

bezierfile = sys.argv[1]

cpts = []

with open(bezierfile, "rb") as csvfile:
    reader = csv.reader(csvfile, delimiter=",")
    first_row = True
    for row in reader:
        if first_row:
            first_row = False
            continue
        row = [float(x) for x in row][1:]
        cpts.append(row)

def fac(n):
    s = 1
    i = 1
    while i<=n:
        s*=i
        i+=1
    return s

def comb(n, i):
    return (fac(n) / fac(n-i)) / fac(i)


def eval(curve, t):
    xx = 0
    yy = 0
    if(t <= 0):
        return curve[0:2]
    elif(t >= 1):
        return curve[len(curve)-2:len(curve)]
    for i in range(8):
        mult = t**i*(1-t)**(7-i)*comb(7,i)
        xx += curve[i*2]*mult
        yy += curve[i*2+1]*mult
    return (xx, yy)

def gencurve(curve):
    t = 0
    x = []
    y = []
    while t<=1.0000000001:
        (xx, yy) = eval(curve, t)
        x.append(xx)
        y.append(yy)
        t += 0.01
    return (x, y)

for idx,curve in enumerate(cpts):
    (x,y) = gencurve(curve)
    plt.plot(x, y, color = colors[idx%len(colors)], lw=2)
    i = 0
    print curve
    while i < len(curve):
        plt.plot(curve[i],curve[i+1], color=colors[idx%len(colors)], marker='o', alpha=0.5)
        i+=2

plt.show()

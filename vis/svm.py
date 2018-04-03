import sys
from matplotlib import pyplot as plt
import numpy as np
obs = []
ro = None
pla = []


while True:
    inp = raw_input().split(" ")
    if(inp[0] == "obs:"):
        obs.append(eval(inp[1]))
    elif(inp[0] == "ro:"):
        ro = eval(inp[1])
    elif(inp[0] == "pla:"):
        pla.append(float(inp[1]))
        pla.append(float(inp[2]))
        pla.append(float(inp[3]))
        break

for o in obs:
    print o
    plt.plot([o[0]], [o[1]], marker='o', color='blue')

plt.plot([ro[0]], [ro[1]], marker='o', color='red')

x = np.linspace(-10, 10)
print pla
y = (pla[2] - pla[0] * x) / pla[1]
plt.plot(x, y)

plt.show()

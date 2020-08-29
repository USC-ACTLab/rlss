import matplotlib.pyplot as plt
import numpy as np

N = 5

gs = (19.71035598705502, 14.56310679611651, 7.727564102564098, 8.195454545454556, 12.088414634146345)
ds = (652.9142394822003, 663.3721682847898, 1559.1009615384628, 5645.312121212118, 206241.16869918705)
trajopt = (9038.961165048544, 9870.440129448758,  13907.296474358975, 25302.192424242425, 61067.34451219511)
vc = (667.2847896440132, 687.9939420685225, 665.7612179487176, 788.1696969696961, 931.1961382113822)

gs_percent = []
ds_percent = []
trajopt_percent = []
vc_percent = []

for i in range(len(gs)):
	total = gs[i] + ds[i] + trajopt[i] + vc[i]
	gs_percent.append(gs[i] / total)
	ds_percent.append(ds[i] / total)
	trajopt_percent.append(trajopt[i] / total)
	vc_percent.append(vc[i] / total)

print("gs percent", gs_percent, ", ds percent", ds_percent, ", trajopt percent", trajopt_percent, ", vc percent", vc_percent)

ind = np.arange(len(gs))
width = 0.35

p1 = plt.bar(ind, gs_percent, width)
p2 = plt.bar(ind, ds_percent, width, bottom = gs_percent)
p3 = plt.bar(ind, trajopt_percent, width, bottom = np.array(ds_percent) + np.array(gs_percent))
p4 = plt.bar(ind, vc_percent, width, bottom = np.array(ds_percent) + np.array(gs_percent) + np.array(trajopt_percent))

plt.ylabel('Percent')
plt.title('Percentage of time spent on each step')
plt.xticks(ind, ('50cm x 50cm x 50cm', '40cm x 40cm x 40cm', '30cm x 30cm x 30cm', '20cm x 20cm x 20cm', '10cm x 10cm x 10cm'))
plt.yticks([0, 1])
plt.legend((p1[0], p2[0], p3[0], p4[0]), ('Goal Selection', 'Discrete Search', 'Trajectory Optimization', 'Temporal Re-scaling'))

plt.show()

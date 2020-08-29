dim = int(input("dim: "))
num_robots = int(input("num_robots: "))
outer_box_min = tuple([float(x) for x in input("outer_box_min: ").strip().split(" ")])
outer_box_max = tuple([float(x) for x in input("outer_box_max: ").strip().split(" ")])
inner_box_min = tuple([float(x) for x in input("inner_box_min: ").strip().split(" ")])
inner_box_max = tuple([float(x) for x in input("inner_box_max: ").strip().split(" ")])

import random

separations = []
for d in range(dim):
    separations.append(((inner_box_min[d] - outer_box_min[d]), (outer_box_max[d] - inner_box_max[d])))

for robot in range(num_robots):
    start_pos = []
    end_pos = []
    for d in range(dim):
        sample = random.random()
        if(sample >= separations[d][0] / (separations[d][0] + separations[d][1])):
            sample -= separations[d][0] / (separations[d][0] + separations[d][1])
            start_pos.append(inner_box_max[d] + separations[d][1] * sample)
        else:
            start_pos.append(outer_box_min[d] + separations[d][0] * sample)


        sample = random.random()
        if(sample >= separations[d][0] / (separations[d][0] + separations[d][1])):
            sample -= separations[d][0] / (separations[d][0] + separations[d][1])
            end_pos.append(inner_box_max[d] + separations[d][1] * sample)
        else:
            end_pos.append(outer_box_min[d] + separations[d][0] * sample)

    d = random.randint(0, d)
    start_pos[d] = outer_box_min[d] + random.random() * (outer_box_max[d] - outer_box_min[d])


    d = random.randint(0, d)
    end_pos[d] = outer_box_min[d] + random.random() * (outer_box_max[d] - outer_box_min[d])

    print("start: ", start_pos, "end: ", end_pos)
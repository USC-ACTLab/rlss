import sys
import json
import math

file_path = sys.argv[1]
f = open(file_path, "r")
log = json.loads(f.read())
f.close()


def aabb_intersects(aabb1, aabb2):
    for i in range(3):
        if aabb1["min"][i] >= aabb2["max"][i]:
            return False
        if aabb1["max"][i] <= aabb2["min"][i]:
            return False

    return True

def aabb_at_position(aabb, position):
    new_min = [0, 0, 0]
    new_max = [0, 0, 0]

    for i in range(3):
        new_min[i] = aabb["min"][i] + position[i]
        new_max[i] = aabb["max"][i] + position[i]


    return {
        "min": new_min,
        "max": new_max
    }

def distance(pos1, pos2):
    distsq = 0
    for i in range(3):
        distsq += (pos2[i] - pos1[i])**2

    return math.sqrt(distsq)

robot_shapes = {}
obstacles = []
robot_positions = {}
total_distance = {}

for shape_dict in log["robot_shapes"]:
    robot_id = shape_dict["robot_id"]
    shape = shape_dict["shape"]

    robot_shapes[robot_id] = shape

collision_count = 0
for frame_dict in log["frames"]:
    if "obstacles" in frame_dict:
        obstacles = frame_dict["obstacles"]

    if "robot_positions" in frame_dict:
        for pos_dict in frame_dict["robot_positions"]:
            robot_id = pos_dict["robot_id"]
            position = pos_dict["position"]

            if(robot_id in robot_positions):
                if not robot_id in total_distance:
                    total_distance[robot_id] = 0
                total_distance[robot_id] += distance(robot_positions[robot_id], position)

            robot_positions[robot_id] = position


    robot_boxes = [aabb_at_position(robot_shapes[robot_id], position) for robot_id, position in robot_positions.items()]

    for rbox in robot_boxes:
        for obs in obstacles:
            if(aabb_intersects(rbox, obs)):
                collision_count += 1

    i = 0
    while i < len(robot_boxes):
        j = i + 1
        while j < len(robot_boxes):
            if(aabb_intersects(robot_boxes[i], robot_boxes[j])):
                collision_count += 1
            j += 1
        i += 1

print("total collisions: ", collision_count)
print("total steps: ", len(log["frames"]))
print("total distance: ", sum(total_distance.values()))
print("avg distance: ", sum(total_distance.values()) / len(total_distance))
from objects.AABB import AABB
from objects.PiecewiseCurve import PiecewiseCurve
import copy
from visualizer.Frame import Frame
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import matplotlib.animation as animation
import matplotlib
#matplotlib.use("Agg")

fig = plt.figure(figsize=(15, 15))
ax = plt.axes(xlim = (-5, 25), ylim = (-2, 20))
ax.set_aspect('equal')

robot_shapes = {}
original_trajectories = {}
trails = {}
trail_concat_interval = 10
robot_colors = {}
robot_transparent_colors = {}
dt = 0
obstacle_color = (0,0,0,1)

robot_shape_plots = {}
original_trajectory_plots = {}
trajectory_plots = {}
obstacle_plots = []
trail_plots = {}


def create_frames(_log):
    global robot_shapes
    global original_trajectories
    global robot_colors
    global robot_transparent_colors
    global dt

    frames = []

    colors = [(0.9019607843137255,0.09803921568627451,0.29411764705882354,1),(0.23529411764705882,0.7058823529411765,0.29411764705882354,1),(1.0,0.8823529411764706,0.09803921568627451,1),(0.0,0.5098039215686274,0.7843137254901961,1),(0.9607843137254902,0.5098039215686274,0.18823529411764706,1),(0.5686274509803921,0.11764705882352941,0.7058823529411765,1),(0.27450980392156865,0.9411764705882353,0.9411764705882353,1),(0.9411764705882353,0.19607843137254902,0.9019607843137255,1),(0.8235294117647058,0.9607843137254902,0.23529411764705882,1),(0.9803921568627451,0.7450980392156863,0.8313725490196079,1),(0.0,0.5019607843137255,0.5019607843137255,1),(0.8627450980392157,0.7450980392156863,1.0,1),(0.6666666666666666,0.43137254901960786,0.1568627450980392,1),(1.0,0.9803921568627451,0.7843137254901961,1),(0.5019607843137255,0.0,0.0,1),(0.6666666666666666,1.0,0.7647058823529411,1),(0.5019607843137255,0.5019607843137255,0.0,1),(1.0,0.8431372549019608,0.7058823529411765,1),(0.0,0.0,0.5019607843137255,1),(0.5019607843137255,0.5019607843137255,0.5019607843137255,1),(1.0,1.0,1.0,1),(0.0,0.0,0.0,1)]
    color_idx = 0


    first_frame = Frame()
    first_frame.setStep(0)

    for robot_shape in _log["robot_shapes"]:
        robot_id = robot_shape["robot_id"]
        shape = robot_shape["shape"]
        robot_colors[robot_id] = colors[color_idx]
        robot_transparent_colors[robot_id] = copy.deepcopy(colors[color_idx])
        cc = (robot_transparent_colors[robot_id][0], robot_transparent_colors[robot_id][1], robot_transparent_colors[robot_id][2], 0.5)
        robot_transparent_colors[robot_id] = cc
        color_idx += 1
        color_idx %= len(colors)

        if "min" in shape and "max" in shape:
            robot_shapes[robot_id] = AABB.fromDictionary(shape)
        else:
            raise Exception("robot shape not recognized for robot", robot_id)

    for traj in _log["original_trajectories"]:
        robot_id = traj["robot_id"]
        ori_traj = traj["trajectory"]
        original_trajectories[robot_id] = PiecewiseCurve.fromDictionary(ori_traj)



    frames.append(first_frame)

    dt = _log["frame_dt"]
    current_time = 0

    for frame_d in _log["frames"]:
        frame = Frame(current_time)
        frame.fromFrame(frames[-1])
        frame.setStep(frame_d["step"])

        # trajectories update
        if "trajectories" in frame_d:
            for traj_dict in frame_d["trajectories"]:
                robot_id = traj_dict["robot_id"]
                traj = PiecewiseCurve.fromDictionary(traj_dict["trajectory"])
                frame.setTrajectory(robot_id, traj)


        # obstacles update
        if "obstacles" in frame_d:
            frame.clearObstacles()
            for shape in frame_d["obstacles"]:
                if "min" in shape and "max" in shape:
                    frame.addObstacle(AABB.fromDictionary(shape))
                else:
                    raise Exception("robot shape not recognized for robot", robot_id)


        # robot positions update
        if "robot_positions" in frame_d:
            for pos_dict in frame_d["robot_positions"]:
                robot_id = pos_dict["robot_id"]
                position = tuple(pos_dict["position"])
                frame.setRobotPosition(robot_id, position)


        frames.append(frame)
        current_time += dt

    return frames

def update(frame):
    global robot_shape_plots
    global robot_shapes
    global robot_colors
    global obstacle_color
    global obstacle_plots
    global trajectory_plots
    global original_trajectories
    global original_trajectory_plots
    global trails
    global last_trail_concat_timestep
    global trail_concat_interval
    global ax
    global dt

    print(frame.step)

    current_time = dt * frame.step

    updated_plots = []
    # print("frame", frame.step)

    for rid, position in frame.robot_positions.items():
        if rid in robot_shape_plots:
            rect = robot_shapes[rid].rect(position, color = robot_colors[rid])
            robot_shape_plots[rid].set_xy(rect.get_xy())
            robot_shape_plots[rid].set_width(rect.get_width())
            robot_shape_plots[rid].set_height(rect.get_height())

            if frame.step % trail_concat_interval == 0:
                pts = np.concatenate((trails[rid], np.array([[position[0]], [position[1]]])), axis = 1)
                trails[rid] = pts
        else:
            robot_shape_plots[rid] = robot_shapes[rid].rect(position, color = robot_colors[rid])
            ax.add_patch(robot_shape_plots[rid])

            trails[rid] = np.array([[position[0]], [position[1]]])

        updated_plots.append(robot_shape_plots[rid])

        if frame.step % trail_concat_interval == 0:
            if rid in trail_plots:
                trail_plots[rid].set_xdata(trails[rid][0, :])
                trail_plots[rid].set_ydata(trails[rid][1, :])
            else:
                trail_plots[rid], = ax.plot(trails[rid][0, :], trails[rid][0, :], '-', color = robot_colors[rid])

        updated_plots.append(trail_plots[rid])


    for rid, traj in original_trajectories.items():
        pts = original_trajectories[rid].pts(current_time)
        if rid in original_trajectory_plots:
            original_trajectory_plots[rid].set_xdata(pts[0, :])
            original_trajectory_plots[rid].set_ydata(pts[1, :])
        else:
            line, = ax.plot(pts[0, :], pts[1, :], '--', color = robot_colors[rid])
            original_trajectory_plots[rid] = line
        updated_plots.append(original_trajectory_plots[rid])



    last_idx = -1
    for idx, obs in enumerate(frame.obstacles):
        if idx < len(obstacle_plots):
            rect = obs.rect((0,0,0), color = obstacle_color)
            obstacle_plots[idx].set_xy(rect.get_xy())
            obstacle_plots[idx].set_width(rect.get_width())
            obstacle_plots[idx].set_height(rect.get_height())
        else:
            rect = obs.rect((0,0,0), color = obstacle_color)
            ax.add_patch(rect)
            obstacle_plots.append(rect)

        updated_plots.append(obstacle_plots[idx])
        last_idx = idx
    last_idx += 1
    while last_idx < len(obstacle_plots):
        obstacle_plots[last_idx].remove()
        del obstacle_plots[last_idx]


    for rid, traj in frame.trajectories.items():
        pts = traj.pts(0)
        if rid in trajectory_plots:
            trajectory_plots[rid].set_xdata(pts[0, :])
            trajectory_plots[rid].set_ydata(pts[1, :])
        else:
            line, = ax.plot(pts[0, :], pts[1, :], '-', color = robot_colors[rid])
            trajectory_plots[rid] = line
        updated_plots.append(trajectory_plots[rid])


    # fig.savefig("images/" + str(frame.step) + ".png")

    return updated_plots

f = open("vis.json", "r")
import json
j = json.loads(f.read())
f.close()

frames = create_frames(j)
print("num frames: ", len(frames))

def init():
    return update(frames[0])

Writer = animation.writers['ffmpeg']
writer = Writer(fps= 1 / dt, bitrate = 8 * 1024)

ani = FuncAnimation(fig, update, frames = frames, blit = True, interval = dt * 3000, repeat = False)

#ani.save("res.mp4", writer = writer, dpi = 150)


plt.show()

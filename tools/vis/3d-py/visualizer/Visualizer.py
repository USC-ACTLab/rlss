from visualizer.Delta import Delta
from objects.AABB import AABB
from objects.Sphere import Sphere
from objects.PiecewiseCurve import PiecewiseCurve
from containers.RobotShapesContainer import RobotShapesContainer
from containers.TrajectoriesContainer import TrajectoriesContainer
import rospy
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
import copy
import queue

class Visualizer:
    def __init__(self, _log, _obstacle_color = ColorRGBA(1, 0, 0, 1)):
        self.command_queue = queue.Queue()
        self.direction = "forward"
        self.paused = False
        self.original_trajectories_enabled = True

        self.log = _log
        colors = [ColorRGBA(0.9019607843137255,0.09803921568627451,0.29411764705882354,1),ColorRGBA(0.23529411764705882,0.7058823529411765,0.29411764705882354,1),ColorRGBA(1.0,0.8823529411764706,0.09803921568627451,1),ColorRGBA(0.0,0.5098039215686274,0.7843137254901961,1),ColorRGBA(0.9607843137254902,0.5098039215686274,0.18823529411764706,1),ColorRGBA(0.5686274509803921,0.11764705882352941,0.7058823529411765,1),ColorRGBA(0.27450980392156865,0.9411764705882353,0.9411764705882353,1),ColorRGBA(0.9411764705882353,0.19607843137254902,0.9019607843137255,1),ColorRGBA(0.8235294117647058,0.9607843137254902,0.23529411764705882,1),ColorRGBA(0.9803921568627451,0.7450980392156863,0.8313725490196079,1),ColorRGBA(0.0,0.5019607843137255,0.5019607843137255,1),ColorRGBA(0.8627450980392157,0.7450980392156863,1.0,1),ColorRGBA(0.6666666666666666,0.43137254901960786,0.1568627450980392,1),ColorRGBA(1.0,0.9803921568627451,0.7843137254901961,1),ColorRGBA(0.5019607843137255,0.0,0.0,1),ColorRGBA(0.6666666666666666,1.0,0.7647058823529411,1),ColorRGBA(0.5019607843137255,0.5019607843137255,0.0,1),ColorRGBA(1.0,0.8431372549019608,0.7058823529411765,1),ColorRGBA(0.0,0.0,0.5019607843137255,1),ColorRGBA(0.5019607843137255,0.5019607843137255,0.5019607843137255,1),ColorRGBA(1.0,1.0,1.0,1),ColorRGBA(0.0,0.0,0.0,1)]
        color_idx = 0

        robot_colors = {}
        robot_transparent_colors = {}

        robot_shapes = RobotShapesContainer()

        for robot_shape in _log["robot_shapes"]:
            robot_id = robot_shape["robot_id"]
            shape = robot_shape["shape"]
            robot_colors[robot_id] = colors[color_idx]
            robot_transparent_colors[robot_id] = copy.deepcopy(colors[color_idx])
            robot_transparent_colors[robot_id].a = 0.5
            color_idx += 1
            color_idx %= len(colors)

            if "min" in shape and "max" in shape:
                robot_shapes.setRobotShape(robot_id, AABB.fromDictionary(shape))
            elif "center" in shape and "radius" in shape:
                robot_shapes.setRobotShape(robot_id, Sphere.fromDictionary(shape))
            else:
                raise Exception("robot shape not recognized for robot", robot_id)

        original_trajectories = TrajectoriesContainer()
        for traj in _log["original_trajectories"]:
            robot_id = traj["robot_id"]
            ori_traj = traj["trajectory"]

            original_trajectories.setRobotTrajectory(robot_id, PiecewiseCurve.fromDictionary(ori_traj))

        init_delta = Delta()
        for rid in robot_shapes.robot_ids():
            initial_position = original_trajectories.eval(rid, 0)
            robot_shapes.updateRobotPosition(rid, initial_position)

        init_delta.addMarkerArray(robot_shapes.toMarkerArray(robot_colors))


        self.deltas = [init_delta]

        self.dt = _log["frame_dt"]
        current_time = 0

        trajectories = TrajectoriesContainer()
        obstacle_markers = []

        for frame in _log["frames"]:
            delta = Delta()

            # trajectories update
            if "trajectories" in frame:
                for traj_dict in frame["trajectories"]:
                    robot_id = traj_dict["robot_id"]
                    traj = PiecewiseCurve.fromDictionary(traj_dict["trajectory"])
                    trajectories.setRobotTrajectory(robot_id, traj)

                last_markers = trajectories.last_markers
                new_markers = trajectories.toMarkerArray(0, robot_colors)
                for marker in new_markers:
                    if marker.id in last_markers:
                        delta.updateMarker(last_markers[marker.id], marker)
                        del last_markers[marker.id]
                    else:
                        delta.addMarker(marker)

                for mid, marker in last_markers.items():
                    delta.removeMarker(marker)


            # obstacles update
            if "obstacles" in frame:
                delta.removeMarkerArray(obstacle_markers)
                obstacle_markers = []
                for shape in frame["obstacles"]:
                    if "min" in shape and "max" in shape:
                        obstacle_markers.append(AABB.fromDictionary(shape).toMarker((0,0,0)))
                    elif "center" in shape and "radius" in shape:
                        obstacle_markers.append(Sphere.fromDictionary(shape).toMarker((0,0,0)))
                    else:
                        raise Exception("robot shape not recognized for robot", robot_id)

                delta.addMarkerArray(obstacle_markers)


            # robot positions update
            if "robot_positions" in frame:
                for pos_dict in frame["robot_positions"]:
                    robot_id = pos_dict["robot_id"]
                    position = tuple(pos_dict["position"])
                    robot_shapes.updateRobotPosition(robot_id, position)

                last_markers = robot_shapes.last_markers
                new_markers = robot_shapes.toMarkerArray(robot_colors)
                for marker in new_markers:
                    if marker.id in last_markers:
                        delta.updateMarker(last_markers[marker.id], marker)
                        del last_markers[marker.id]
                    else:
                        delta.addMarker(marker)

                for mid, marker in last_markers.items():
                    delta.removeMarker(marker)


            # original trajectories update
            # last_markers = original_trajectories.last_markers
            # new_markers = original_trajectories.toMarkerArray(current_time, robot_transparent_colors)
            # for marker in new_markers:
            #     if marker.id in last_markers:
            #         delta.updateMarker(last_markers[marker.id], marker)
            #         del last_markers[marker.id]
            #     else:
            #         delta.addMarker(marker)
            #
            # for mid, marker in last_markers.items():
            #     delta.removeMarker(marker)

            self.deltas.append(delta)
            current_time += self.dt

    def apply_delta(self, delta, direction, pub):
        if direction == "forward":
            markers = delta.forward()
        elif direction == "backward":
            markers = delta.backward()
        else:
            raise Exception("direction not valid")

        pub.publish(MarkerArray(markers))

    def _loop(self):
        rate = rospy.Rate(1/self.dt)
        pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size = 10)
        pub.publish(MarkerArray([]))

        for i in range(100):
            rate.sleep()

        idx = 0
        while not rospy.is_shutdown():
            while True:
                try:
                    command = self.command_queue.get(block = False)
                    if command["action"] == "backward":
                        self.direction = "backward"
                    elif command["action"] == "forward":
                        self.direction = "forward"
                    elif command["action"] == "step":
                        if self.direction == "forward":
                            target = idx + command["count"]
                            target = min(len(self.deltas)-1, target)
                            while idx < target:
                                self.apply_delta(self.deltas[idx], self.direction, pub)
                                idx += 1
                        elif self.direction == "backward":
                            target = idx - command["count"]
                            target = max(0, target)
                            while idx > target:
                                self.apply_delta(self.deltas[idx], self.direction, pub)
                                idx -= 1

                    elif command["action"] == "pause":
                        self.paused = True
                    elif command["action"] == "unpause":
                        self.paused = False
                    elif command["action"] == "exit":
                        return
                except queue.Empty as e:
                    break

            if not self.paused:
                if self.direction == "forward":
                    self.apply_delta(self.deltas[idx], "forward", pub)
                    idx += 1
                    idx = min(len(self.deltas)-1, idx)
                elif self.direction == "backward":
                    self.apply_delta(self.deltas[idx], "backward", pub)
                    idx -= 1
                    idx = max(0, idx)
                else:
                    raise Exception("direction not valid")
            rate.sleep()

    def run(self):
        self._loop()

if __name__ == "__main__":
    import json
    f = open("out.json", "r")
    j = json.loads(f.read())
    f.close()

    vis = Visualizer(j)
    vis.run()
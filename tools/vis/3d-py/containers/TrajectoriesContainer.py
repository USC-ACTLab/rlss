from std_msgs.msg import ColorRGBA

class TrajectoriesContainer:
    def __init__(self):
        self.trajectories = {}
        self.last_markers = {}

    def robot_ids(self):
        return [id for id in self.trajectories]

    def setRobotTrajectory(self, _robot_id, _trajectory):
        self.trajectories[_robot_id] = _trajectory


    def eval(self, _robot_id, _param):
        return self.trajectories[_robot_id].eval(_param)

    def toMarkerArray(self, _param_from, colors, _frame_id = "map"):
        markers = []
        for rid, traj in self.trajectories.items():
            markers.extend(traj.toMarkerArray(_param_from, _frame_id, colors[rid]))

        self.last_markers = {}
        for marker in markers:
            self.last_markers[marker.id] = marker

        return markers
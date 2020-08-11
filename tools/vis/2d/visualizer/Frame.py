import copy

class Frame:
    def __init__(self, _current_time = 0):
        self.current_time = _current_time
        self.robot_positions = {}
        self.trajectories = {}
        self.obstacles = []
        self.step = 0

    def fromFrame(self, frame):
        self.robot_positions = copy.deepcopy(frame.robot_positions)
        self.trajectories = copy.deepcopy(frame.trajectories)
        self.obstacles = frame.obstacles

    def setStep(self, step):
        self.step = step

    def setRobotPosition(self, robot_id, position):
        self.robot_positions[robot_id] = position

    def setTrajectory(self, robot_id, traj):
        self.trajectories[robot_id] = traj

    def clearObstacles(self):
        self.obstacles = []

    def addObstacle(self, shape):
        self.obstacles.append(shape)

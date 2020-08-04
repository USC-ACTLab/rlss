from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA

class RobotShapesContainer:
    def __init__(self):
        self.shapes = {}
        self.positions = {}
        self.last_markers = {}

    def robot_ids(self):
        return [id for id in self.shapes]

    def setRobotShape(self, _robot_id, _shape):
        self.shapes[_robot_id] = _shape
        self.positions[_robot_id] = (0, 0, 0)

    def updateRobotPosition(self, _robot_id, _position):
        self.positions[_robot_id] = _position
        print(_position)

    def toMarkerArray(self, colors,  frame_id = "map"):
        markers = []

        for rid, shape in self.shapes.items():
            markers.append(shape.toMarker(self.positions[rid], frame_id, colors[rid]))

        self.last_markers = {}
        for marker in markers:
            self.last_markers[marker.id] = marker

        return markers
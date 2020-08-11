from id import id
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from id import id
import numpy as np

class SegmentList:
    def __init__(self):
        self.points = []
        self.id = id.next()

    def addPt(self, point):
        self.points.append(point)

    def toMarker(self, frame_id = "map", color = ColorRGBA(1, 0, 0, 1)):
        marker = Marker()
        marker.type = marker.LINE_STRIP
        marker.header.frame_id = frame_id
        marker.action = marker.ADD
        marker.scale = Vector3(0.03, 0, 0)
        marker.color = color
        marker.id = self.id
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        marker.points = [Point(pt[0], pt[1], pt[2]) for pt in self.points]
        return marker

    def toMarkerArray(self, frame_id = "map", color = ColorRGBA(1, 0, 0, 1)):
        return MarkerArray([self.toMarker(frame_id, color)])

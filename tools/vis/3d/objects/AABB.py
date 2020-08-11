from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import rospy
from id import id

class AABB:
    @staticmethod
    def fromDictionary(_dict):
        return AABB(tuple(_dict["min"]), tuple(_dict["max"]))

    def __init__(self, _min, _max):
        assert(type(_min) == tuple and len(_max) == 3)
        assert(type(_max) == tuple and len(_min) == 3)
        self.min = _min
        self.max = _max
        self.id = id.next()

    def toMarker(self, position, frame_id = "map", color = ColorRGBA(0.2, 0.2, 0.2, 1)):
        center = tuple([(m+M) / 2.0 + p for m,M,p in zip(self.min, self.max, position)])
        scale = tuple([(M-m) for m, M in zip(self.min, self.max)])
        assert(len(center) == 3)
        assert(len(scale) == 3)

        marker = Marker()
        marker.type = marker.CUBE
        marker.header.frame_id = frame_id
        marker.action = marker.ADD
        marker.pose.position = Point(center[0], center[1], center[2])
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(scale[0], scale[1], scale[2])
        marker.color = color
        marker.id = self.id

        return marker

    def toMarkerArray(self, position, frame_id = "map", color = ColorRGBA(1, 0, 0, 1)):
        return MarkerArray([self.toMarker(position, frame_id, color)])


if __name__ == "__main__":
    rospy.init_node("aabb_test")
    pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        aabb = AABB((0,0,0), (3,3,3))
        marker = aabb.toMarker((0,0,0))
        pub.publish(MarkerArray([marker]))
        rate.sleep()
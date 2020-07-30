from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import rospy
from id import id

class Sphere:
    def __init__(self, _center, _radius):
        assert(type(_center) == tuple and len(_center) == 3)
        assert(type(_radius) == float or type(_radius) == int)

        self.center = _center
        self.radius = _radius
        self.id = id.next()

    def toMarker(self, frame_id = "map", color = ColorRGBA(1, 0, 0, 1)):
        scale = (2 * self.radius, 2 * self.radius, 2 * self.radius)

        marker = Marker()
        marker.type = marker.SPHERE
        marker.header.frame_id = frame_id
        marker.action = marker.ADD
        marker.pose.position = Point(self.center[0], self.center[1], self.center[2])
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(scale[0], scale[1], scale[2])
        marker.color = color
        marker.id = self.id

        return marker


if __name__ == "__main__":
    rospy.init_node("sphere_test")
    pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        sphere = Sphere((0,0,0), 2)
        marker = sphere.toMarker()
        pub.publish(MarkerArray([marker]))
        rate.sleep()
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import rospy
from id import id
import sympy


class BezierCurve:
    def __init__(self, _max_param, _cpts):
        assert(type(_max_param) == float or type(_max_param) == int)
        assert(type(_cpts) == list)
        for cpt in _cpts:
            assert(type(cpt) == tuple)

        self.max_parameter = _max_param
        self.controlpoints = _cpts

        self.id = id.next()

    def toMarker(self, param_from = 0, frame_id = "map", color = ColorRGBA(1, 0, 0, 1)):
        marker = Marker()
        marker.type = marker.LINE_STRIP
        marker.header.frame_id = frame_id
        marker.action = marker.ADD
        marker.scale = Vector3(0.01, 0, 0)
        marker.color = color
        marker.id = self.id
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        t = param_from
        points = []
        while t < self.max_parameter:
            pt = (0, 0, 0)
            d = len(self.controlpoints) - 1
            tT = t / self.max_parameter
            for i, cpt in enumerate(self.controlpoints):
                basis = sympy.binomial(d, i) * (tT ** i) * ((1-tT) ** (d-i))
                pt = (pt[0] + cpt[0] * basis, pt[1] + cpt[1] * basis, pt[2] + cpt[2] * basis)
            points.append(Point(pt[0], pt[1], pt[2]))
            t += 0.1

        end_pt = self.controlpoints[-1]
        points.append(Point(end_pt[0], end_pt[1], end_pt[2]))

        marker.points = points

        return marker


if __name__ == "__main__":
    rospy.init_node("bezier_test")
    pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size = 10)
    duration = 4.78978
    control_points= [
        (-3.51892,1.88924,0.75),
        (-3.51892,1.88924,0.75),
        (-3.51892,1.88924,0.75),
        (-3.51892,1.88924,0.75),
        (-2.2442192974,1.37519264943,0.673804284093),
        (-1.79448197019,1.15927033917,0.812013024357),
        (-1.20297981168,0.880590763017,0.968507627773),
        (-0.611415798577,0.601931825628,1.1250072342)
    ]
    curve = BezierCurve(duration, control_points)

    rate = rospy.Rate(10) # 1hz
    t = 0
    while t < duration and not rospy.is_shutdown():
        marker = curve.toMarker(t)

        pub.publish(MarkerArray([marker]))
        rate.sleep()
        t += 0.1
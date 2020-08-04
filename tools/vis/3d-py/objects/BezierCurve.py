from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import rospy
from id import id
import sympy
import numpy as np
import bezier


class BezierCurve:

    @staticmethod
    def fromDictionary(_dict):
        assert(_dict["type"] == "bezier")
        cpts = []
        for cpt in _dict["controlpoints"]:
            cpts.append(tuple(cpt))
        return BezierCurve(_dict["max_parameter"], cpts)

    def __init__(self, _max_param, _cpts):
        assert(type(_max_param) == float or type(_max_param) == int)
        assert(type(_cpts) == list)
        for cpt in _cpts:
            assert(type(cpt) == tuple)

        self.max_parameter = _max_param
        self.controlpoints = np.array(_cpts)
        self.curve = bezier.Curve(np.transpose(self.controlpoints), degree = len(_cpts) - 1)

        self.id = id.next()

    def eval(self, param):
        assert(param <= self.max_parameter and param >= 0)
        return tuple(np.transpose(self.curve.evaluate(param / self.max_parameter))[0])

    def toMarker(self, param_from = 0, frame_id = "map", color = ColorRGBA(1, 0, 0, 1)):
        marker = Marker()
        marker.type = marker.LINE_STRIP
        marker.header.frame_id = frame_id
        marker.action = marker.ADD
        marker.scale = Vector3(0.03, 0, 0)
        marker.color = color
        marker.id = self.id
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        num_samples = int((self.max_parameter - param_from) / 0.1) + 1
        if num_samples == 1:
            num_samples = 2

        vals = np.linspace(param_from / self.max_parameter, 1.0, num_samples)
        pts = np.transpose(self.curve.evaluate_multi(vals))
        marker.points = [Point(pt[0], pt[1], pt[2]) for pt in pts]
        return marker

    def toMarkerArray(self, param_from = 0, frame_id = "map", color = ColorRGBA(1, 0, 0, 1)):
        return MarkerArray([self.toMarker(param_from, frame_id, color)])

def test():
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

    eval_period = 0.01
    rate = rospy.Rate(1 / eval_period)
    t = 0
    while t < duration and not rospy.is_shutdown():
        marker = curve.toMarker(t)

        pub.publish(MarkerArray([marker]))
        rate.sleep()
        t += eval_period

if __name__ == "__main__":
    import cProfile
    cProfile.run('test()')
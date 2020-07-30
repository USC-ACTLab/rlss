from BezierCurve import BezierCurve
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import rospy

class PiecewiseTrajectory:
    def __init__(self, _pieces):
        assert(type(_pieces) == list)
        for piece in _pieces:
            assert(type(piece) == BezierCurve)

        self.pieces = _pieces
        self.max_parameter = sum([piece.max_parameter for piece in _pieces])


    def toMarkerArray(self, param_from = 0, frame_id = "map", color = ColorRGBA(1, 0, 0, 1)):
        i = 0
        while param_from > self.pieces[i].max_parameter:
            param_from -= self.pieces[i].max_parameter
            i += 1


        markers = []
        while i < len(self.pieces):
            markers.append(self.pieces[i].toMarker(param_from, frame_id, color))
            param_from = 0
            i += 1

        return MarkerArray(markers)


if __name__ == "__main__":
    rospy.init_node("piecewise_test")
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
    piece1 = BezierCurve(duration, control_points)

    duration = 4.81254
    control_points = [
        (-0.611505,0.601895,1.125),
        (-0.0172071854228,0.321883547651,1.28223668189),
        (0.577090629154,0.0418720953028,1.43947336378),
        (1.02760307657,-0.174474089868,1.57816455423),
        (2.29590940091,-0.685450298368,1.4999997525),
        (2.2959012435,-0.685449101747,1.49999949992),
        (2.29587699784,-0.685452982346,1.50000027797),
        (2.29582118084,-0.685492687181,1.50000654961)
    ]
    piece2 = BezierCurve(duration, control_points)

    traj = PiecewiseTrajectory([piece1, piece2])
    rate = rospy.Rate(10)

    t = 0
    while t < traj.max_parameter and not rospy.is_shutdown():
        markers = traj.toMarkerArray(t)
        pub.publish(markers)

        t += 0.1
        rate.sleep()
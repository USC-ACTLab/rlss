from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import rospy
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


    def eval(self, param):
        assert(param <= self.max_parameter and param >= 0)
        return tuple(np.transpose(self.curve.evaluate(param / self.max_parameter))[0])

    def pts(self, _param_from):
        num_samples = int((self.max_parameter - _param_from) / 0.2) + 1
        if num_samples == 1:
            num_samples = 2

        vals = np.linspace(_param_from / self.max_parameter, 1.0, num_samples)
        pts = self.curve.evaluate_multi(vals)

        return pts

    def draw(self, _ax, _param_from):
        pts = self.pts(_param_from)
        _ax.plot(pts[0, :], pts[1, :])



if __name__ == "__main__":
    duration = 4.78978
    control_points= [
        (-3.51892,1.88924),
        (-3.51892,1.88924),
        (-3.51892,1.88924),
        (-3.51892,1.88924),
        (-2.2442192974,1.37519264943),
        (-1.79448197019,1.15927033917),
        (-1.20297981168,0.880590763017),
        (-0.611415798577,0.601931825628)
    ]

    control_points = [
        (-0.611505,0.601895),
        (-0.0172071854228,0.321883547651),
        (0.577090629154,0.0418720953028),
        (1.02760307657,-0.174474089868),
        (2.29590940091,-0.685450298368),
        (2.2959012435,-0.685449101747),
        (2.29587699784,-0.685452982346),
        (2.29582118084,-0.685492687181)
    ]
    curve = BezierCurve(duration, control_points)

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1)
    curve.draw(ax, 0)

    plt.show()
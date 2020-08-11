from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import rospy
from matplotlib.patches import Rectangle

class AABB:
    @staticmethod
    def fromDictionary(_dict):
        return AABB(tuple(_dict["min"]), tuple(_dict["max"]))

    def __init__(self, _min, _max):
        assert(type(_min) == tuple and len(_max) == 2)
        assert(type(_max) == tuple and len(_min) == 2)
        self.min = _min
        self.max = _max

    def rect(self, _position, color):
        anchor = (self.min[0] + _position[0], self.min[1] + _position[1])
        scale = (self.max[0] - self.min[0], self.max[1] - self.min[1])

        rect = Rectangle(anchor, scale[0], scale[1], color = color)
        return rect

    def draw(self, _ax, _position):
        rect = self.rect(_position)
        _ax.add_patch(rect)

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    aabb = AABB((-2,-2), (3,3))

    fig, ax = plt.subplots(1)
    aabb.draw(ax, (5, 5))

    plt.show()
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def create_point(x=0,y=0,z=0):
   pt1=Point()
   pt1.x = x
   pt1.y = y
   pt1.z = z
   return pt1

def create_color(r,g,b,a):
    res = ColorRGBA()
    res.r,res.g,res.b,res.a=r,g,b,a
    return res

def center(p1,p2):
    cp =  [(i+j)/2.0 for i,j in zip(p1,p2)]
    return create_point(cp[0],cp[1],cp[2])

def getDims(min_,max_):
    return [(j-i) for i,j in zip(min_,max_)]




"""
It is assumed that all cells have same scale
Occupied cells is in the form of:
[[min_xyz_1],[max_xyz_1],[min_xyz_2],[max_xyz_2]]
"""
class OccupiedCells(object):

    def __init__(self,points = []):
        self.points  = points

    def to_marker(self,frame_id="map"):
        if len(self.points)>0:
            wx,wy,wz = getDims(self.points[0],self.points[1])
        else:
            wx,wy,wz = 0,0,0
        marker = Marker()
        clr_ = create_color(1.0,0,0,1.0)
        pts = [center(mn,mx) for mn,mx in zip(self.points[0::2], self.points[1::2])]
        colors = [clr_ for x in pts]
        marker.type = marker.CUBE_LIST
        marker.action = marker.ADD
        marker.scale.x = wx
        marker.scale.y = wy
        marker.scale.z = wz
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.points = pts
        marker.colors = colors
        marker.header.frame_id = frame_id
        return marker


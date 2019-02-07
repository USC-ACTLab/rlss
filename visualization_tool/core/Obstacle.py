from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def create_point(x=0,y=0,z=0):
   pt1=Point()
   pt1.x = x
   pt1.y = y
   pt1.z = z
   return pt1

"""
Points are in the form of [[px1,py1,pz1],[px2,py2,pz2]...], and has to be
in the correct order for NOW. 
#TODO: add Convex Hull
"""

class Obstacle(object):
    
    def __init__(self,points=[]):
        self.points=points

    def to_marker(self,frame_id="map"):
        pts = []
        for i in range(len(self.points)-1):
            pt1 = self.points[i]
            pt2 = self.points[i+1]
            pts.append(create_point(pt1[0],pt1[1],pt1[2]))
            pts.append(create_point(pt2[0],pt2[1],pt2[2]))
        pt1 = self.points[-1]
        pt2 = self.points[0]
        pts.append(create_point(pt1[0],pt1[1],pt1[2]))
        pts.append(create_point(pt2[0],pt2[1],pt2[2]))
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = pts
        return marker

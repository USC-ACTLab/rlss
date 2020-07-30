from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import json

def create_point(x=0,y=0,z=0):
   pt1=Point()
   pt1.x = x
   pt1.y = y
   pt1.z = z
   return pt1


"""
Created directly from discrete_path key of JSON.
"""

class DiscretePath(object):
    
    def __init__(self,json=None):
        self.points= [create_point(x=p[0],y=p[1],z=p[2]) for p in json]
        if len(self.points)>=3:
            pts = []
            pts.append(self.points[0])
            for x in self.points[1:-1]:
                pts.append(x)
                pts.append(x)
            pts.append(self.points[-1])
            self.points = pts
    def to_marker(self,frame_id="map"):
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
        marker.points = self.points
        return marker

if __name__=="__main__":
    with open("../data/samplex.json",'r') as fl:
        js = json.load(fl)
        oc = DiscretePath(js["frames"][0]["trajectories"][0]["discrete_path"])
        print(oc.to_marker())



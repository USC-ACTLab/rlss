from visualization_msgs.msg import Marker

class Robot(object):

    def __init__(self,id=0,x=0,y=0,z=0,radius=0.25):
        self.id = id 
        self.x=x
        self.y=y
        self.z=z
        self.radius = radius 

    
    def setPosition(self,x=0,y=0,z=0):
        self.x=x
        self.y=y
        self.z=z

    def to_marker(self,frame_id="map"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = self.radius
        marker.scale.y = self.radius
        marker.scale.z = self.radius
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        marker.pose.orientation.w = 1.0
        return marker



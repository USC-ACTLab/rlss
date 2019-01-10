from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy
import math
import numpy as np
from tf.transformations import quaternion_from_euler

def create_point(x=0,y=0,z=0):
   pt1=Point()
   pt1.x = x
   pt1.y = y
   pt1.z = z
   return pt1

"""
Hyperplane container class for visualization. Supports both 2d and 3d marker output
@param normal normal is a 3 element iterable type.
@param distance distance in the direction of normal
"""
class Hyperplane(object):

    E = 0.000001

    def __init__(self,normal,distance):
        #TODO: add input control
        self.normal = [x if x!=0 else Hyperplane.E for x in normal]
        self.distance = distance


    """
    Generates 3d marker output from given hyperplane as a cubic volume.
    @param frame_id frame id of output marker
    @param width width of the marker
    @param height height of the marker
    @param depth depth of the marker
    @param color color of the marker. TODO: Random is possible if color generator is passed
    """
    def to_marker_3d(self,frame_id="map",width=1.0,height=1.0,depth=0.1,color=None):
        #TODO: add color
        normal = np.array(self.normal)
        unit_v = normal/np.linalg.norm(normal)
        ux,uy,uz = unit_v
        center = unit_v*self.distance
        xp,yp,zp = center
        yaw = math.atan2(yp,xp)
        pitch = math.atan2(float(zp),math.sqrt(xp**2+yp**2))
        quat = quaternion_from_euler(0,math.pi/2-pitch,yaw)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = width
        marker.scale.y = height
        marker.scale.z = depth
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = xp
        marker.pose.position.y = yp
        marker.pose.position.z = zp
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        return marker


    def to_marker_2d(self,frame_id="map"):
        #TODO: add variable length
        nx,ny,_ = self.normal
        h_norm = np.array([nx,ny])
        st_pt = (h_norm/np.linalg.norm(h_norm))*self.distance
        diff = np.array([-st_pt[1],st_pt[0]])
        p1 = st_pt+diff
        p2 = st_pt-diff
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.2
        marker.color.a = 0.3
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        pt = []
        pt.append(create_point(p1[0],p1[1]))
        pt.append(create_point(p2[0],p2[1]))
        marker.points = pt 
        return marker

    def get_normal_marker(self,frame_id="map"):
        #TODO: add variable length
        normal = np.array(self.normal)
        unit_v = normal/np.linalg.norm(normal)
        center = unit_v*self.distance
        xp,yp,zp = center
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        pt = []
        pt.append(create_point(0,0,0))
        pt.append(create_point(xp,yp,zp))
        marker.points = pt 
        return marker
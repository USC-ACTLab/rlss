import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import math
import numpy as np
from tf.transformations import quaternion_from_euler
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker,queue_size=50)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100


h_norm = np.array([1,2])
h_dist = math.sqrt(5)

st_pt = (h_norm/np.linalg.norm(h_norm))*h_dist
diff = np.array([-st_pt[1],st_pt[0]])

p1 = st_pt+diff
p2 = st_pt-diff



def create_point(x,y):
   pt1=Point()
   pt1.x = x
   pt1.y = y
   return pt1


direction = np.array([1,0.0001,0.0001])
unit_v = direction/np.linalg.norm(direction)
dist = math.sqrt(1)
center = unit_v*dist
xp = center[0]
yp = center[1]
zp = center[2]

yaw = math.atan(xp/(-yp))
pitch = math.atan(math.sqrt(xp**2+yp**2)/zp)
quat = quaternion_from_euler(0,pitch,yaw)



while not rospy.is_shutdown():

   marker = Marker()
   marker.header.frame_id = "/world"
   marker.type = marker.CUBE
   marker.action = marker.ADD
   marker.scale.x = 1.0
   marker.scale.y = 0.1
   marker.scale.z = 1.0
   marker.color.a = 1.0
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

   #pt = []
   #pt.append(create_point(p1[0],p1[1]))
   #pt.append(create_point(p2[0],p2[1]))
   #marker.points = pt 
   publisher.publish(marker)
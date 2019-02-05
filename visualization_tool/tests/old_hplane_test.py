import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import math
import numpy as np
  
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker)

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

while not rospy.is_shutdown():

   marker = Marker()
   marker.header.frame_id = "/world"
   marker.type = marker.LINE_LIST
   marker.action = marker.ADD
   marker.scale.x = 0.01
   marker.scale.y = 0.01
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
   pt.append(create_point(p1[0],p1[1]))
   pt.append(create_point(p2[0],p2[1]))
   marker.points = pt 
   publisher.publish(marker)
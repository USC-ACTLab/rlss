import rospy
from core.Obstacle import Obstacle
from visualization_msgs.msg import MarkerArray
import time
from tf import TransformBroadcaster
from tf import transformations as tfs
import math
import random 
import numpy as np 
from core.FrameContainer import FrameContainer

def get_marker_pose(marker):
    r1 = (marker.pose.position.x,marker.pose.position.y,marker.pose.position.z)
    r2 = (marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w)
    return r1,r2

rospy.init_node("test")
rp = rospy.Publisher("marker_test",MarkerArray,queue_size=10)
fc = FrameContainer("/home/alpcevikel/mr-trajectory-replanning/visualization_tool/data/twor.json")
fr,tm = fc.getFrame(0)
fr2,tm2 = fc.getFrame(1)
wh = True
i=0
while not rospy.is_shutdown():
    if wh:
        m = fr.toMarkerArray(timestamp=rospy.Time.now())
        rp.publish(m)
        wh=False
    else:
        m = fr2.toMarkerArray(timestamp=rospy.Time.now())
        rp.publish(m)
        wh=True        
    time.sleep(1)
    i+=1



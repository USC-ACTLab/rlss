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
import sys

if(len(sys.argv)<2):
    exit("Usage: frame_test.py <json-path>")

jsn_path = sys.argv[1]

def get_marker_pose(marker):
    r1 = (marker.pose.position.x,marker.pose.position.y,marker.pose.position.z)
    r2 = (marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w)
    return r1,r2

rospy.init_node("test")
rp = rospy.Publisher("marker_test",MarkerArray,queue_size=10)
fc = FrameContainer(jsn_path)
i = 0

while not rospy.is_shutdown():
    try:
        fr,tm = fc.getFrame(i)
        m = fr.toMarkerArray(timestamp = rospy.Time.now())
        rp.publish(m)
        time.sleep(fc.frame_dt)
        i += 1
    except IndexError:
        i = 0

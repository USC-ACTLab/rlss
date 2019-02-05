import rospy
from core.Robot import Robot
from visualization_msgs.msg import Marker
import time
from tf import TransformBroadcaster
from tf import transformations as tfs
import math
import random 
import numpy as np 


def get_marker_pose(marker):
    r1 = (marker.pose.position.x,marker.pose.position.y,marker.pose.position.z)
    r2 = (marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w)
    return r1,r2

rospy.init_node("test")
rp = rospy.Publisher("marker_test",Marker,queue_size=10)
xp = rospy.Publisher("marker_test_2",Marker,queue_size=10)
rb = Robot(x=0,y=0,z=0,radius=0.25)
ps = 0
tfb = TransformBroadcaster()
while not rospy.is_shutdown():
    rb.setPosition(ps,ps,ps)
    m = rb.to_marker()
    m.header.stamp = rospy.Time.now()
    rp.publish(m)
    time.sleep(0.25)
    ps+=0.25




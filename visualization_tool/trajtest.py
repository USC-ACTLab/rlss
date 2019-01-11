import rospy
from Trajectory import Trajectory
from Hyperplane import Hyperplane
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
tfb = TransformBroadcaster()
while not rospy.is_shutdown():
    tre = Trajectory(csv_file_3d="conebezier.csv")
    m = tre.to_marker(0,granularity=100)
    m.header.stamp = rospy.Time.now()
    #rp.publish(m)
    hp = Hyperplane((2,2,2),2)
    m = hp.to_marker_3d(width=5,height=5,depth=0.05)
    rp.publish(m)
    m2 = hp.get_normal_marker()
    xp.publish(m2)
    pos,ori = get_marker_pose(m)
    tfb.sendTransform(pos,ori,rospy.Time.now(),"plane","map")
    time.sleep(3)




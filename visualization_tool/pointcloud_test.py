import rospy
from core.Obstacle import Obstacle
from core.Robot import Robot
from core.Hyperplane import Hyperplane
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import time
from tf import TransformBroadcaster
from tf import transformations as tfs
import math
import random
import numpy as np
import sys

if(len(sys.argv) < 2):
    exit("Usage python pointcloud_test.py <txt-file>")

f = open(sys.argv[1], "r")
lines = f.readlines()
f.close()


rospy.init_node("test")
rp = rospy.Publisher("marker_test2",MarkerArray,queue_size=10)
while not rospy.is_shutdown():
    markers = []
    hp_added = False
    for line in lines:
        line = line.strip()
        tokens = line.split(" ")
        print tokens
        m = None
        if(len(tokens) < 4):
            continue
        if tokens[0] == "vc":
            obj = Robot(x=float(tokens[1]), y=float(tokens[2]), z=float(tokens[3]), radius = 0.1)
            m = obj.to_marker()
        elif tokens[0] == "hp":
            obj = Hyperplane(normal=(float(tokens[1]), float(tokens[2]), float(tokens[3])), distance = -float(tokens[4]))
            m = obj.to_marker(width = 100, height = 100, depth=0.01)
        else:
            pass
        if m!= None:
            markers.append(m)
    msg = MarkerArray()
    msg.markers = markers
    sq = 0
    for ms in markers:
        ms.id = sq
        sq+=1
    rp.publish(msg)
    time.sleep(1)

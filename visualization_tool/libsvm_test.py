import rospy
from core.Obstacle import Obstacle
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Point
import time
from tf import TransformBroadcaster
from tf import transformations as tfs
import math
import random
import numpy as np
from core.Hyperplane import Hyperplane
import sys


def create_point(x=0,y=0,z=0):
   pt1=Point()
   pt1.x = x
   pt1.y = y
   pt1.z = z
   return pt1

sys.argv.append("fes")
if(len(sys.argv)<2):
    exit("Usage: libsvm_test.py fname")

def parseRow(row,rho,label):
    lst = row.split(" ")
    sv_coef = float(lst[0])
    b = -1*rho
    w = []
    for el in lst[1:-1]:
        w.append(float(el.split(":")[1]))
    w = [x*sv_coef for x in w]
    dst = abs(b)/math.sqrt(w[0]**2 + w[1]**2+ w[2]**2)
    if label==2 :
        w = [-1*x for x in w]
        b = -1*b
    return Hyperplane(w,dst)

path = "/home/alpcevikel/libsvm/"
fname = sys.argv[1]
add = False
label = 1
hyperplanes = []
with open(path+fname+".model","r") as f:
    for lin in f.readlines():
        if lin[0:3]=="rho":
            rho = float(lin.split(" ")[1])
            continue
        elif lin[0:5]=="nr_sv":
            num_c1 = int(lin.split(" ")[1])
        if lin[0:2] == "SV":
            add=True
            continue
        elif add:
            hyperplanes.append(parseRow(lin,rho,label))
            if num_c1>0:
                num_c1-=1
                if num_c1==0: label =2

pts  = []
with open(path+fname,"r") as f:
    for lin in f.readlines():
        lsx = lin.strip().split(" ")
        px_ = float(lsx[1].split(":")[1])
        py_ = float(lsx[2].split(":")[1])
        pz_ = float(lsx[3].split(":")[1])
        pts.append(create_point(x=px_,y=py_,z=pz_))
mpts=Marker()
mpts.header.frame_id = "map"
mpts.type = Marker.POINTS
mpts.action = Marker.ADD
mpts.pose.orientation.w  = 1.0
mpts.scale.x = 0.1
mpts.scale.y = 0.1
mpts.scale.z = 0.1
mpts.points = pts
mpts.color.r = 1.0
mpts.color.a = 1.0
mpts.id = 1
rospy.init_node("test")
rp = rospy.Publisher("marker_test",MarkerArray,queue_size=10)
for h in hyperplanes:
    print h.normal," ",h.distance

alt = True
while not rospy.is_shutdown():
        m = MarkerArray()
        mpts.header.stamp = rospy.Time.now()
        m.markers = [mpts]
        '''
        for h in hyperplanes:
            print h.normal," ",h.distance
            m.markers.append(h.to_marker())
        
        '''
        if alt:
            m.markers.append(hyperplanes[0].to_marker())
            #alt = False
        else:
            m.markers.append(hyperplanes[1].to_marker())
            alt = True
        rp.publish(m)
        time.sleep(2)

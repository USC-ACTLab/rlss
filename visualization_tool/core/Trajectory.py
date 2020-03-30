from core.CurvePiece import CurvePiece
import csv
import bezier
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

"""
Utility functions that creates point message with x,y,z points
"""
def creatept(x,y,z):
    pt = Point()
    pt.x = x
    pt.y = y
    pt.z = z
    return pt

"""
Class that holds multiple curve pieces as trajectory. Constructed directly from trajectory key of the
json object, if json key is passed. Contstructed from csv file if csv param is passed.Start is the start time of the first trajectory, end is the end time of the last trajectory
"""
class Trajectory(object):
    #TODO: verify that trajectories are given in sorted order
    #TODO: enforce continuity
    def __init__(self,json_file=None,csv_file_2d=None,csv_file_3d=None):
        if json_file:
            self.curve_pieces = [CurvePiece(p) for p in json_file]
            self.start = self.curve_pieces[0].start
            self.end = self.curve_pieces[-1].end

        elif csv_file_2d:
            self.curve_pieces = []
            with open(csv_file_2d, 'rb') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=',')
                first = True
                curtime = 0
                for row in spamreader:
                    if first:
                        first = False
                        continue
                    dur = float(row[0])
                    cpts = row[1:]
                    curpiece = []
                    for i,k in zip(cpts[0::2], cpts[1::2]):
                        curpiece.append([float(i),float(k),float(0)])
                    crv =  bezier.Curve.from_nodes(np.asfortranarray(curpiece).T.astype("double"))
                    cp = CurvePiece()
                    cp.start = curtime
                    cp.end = curtime+dur
                    cp.curve = crv
                    self.curve_pieces.append(cp)
                    curtime+=dur
            self.start = self.curve_pieces[0].start
            self.end = self.curve_pieces[-1].end

        elif csv_file_3d:
            self.curve_pieces = []
            with open(csv_file_3d, 'rb') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=',')
                first = True
                curtime = 0
                for row in spamreader:
                    if first:
                        first = False
                        continue
                    dur = float(row[0])
                    cpts = row[1:]
                    curpiece = []
                    for i,k,j in zip(cpts[0::3], cpts[1::3], cpts[2::3]):
                        curpiece.append([float(i),float(k),float(j)])
                    crv =  bezier.Curve.from_nodes(np.asfortranarray(curpiece).T.astype("double"))
                    cp = CurvePiece()
                    cp.start = curtime
                    cp.end = curtime+dur
                    cp.curve = crv
                    self.curve_pieces.append(cp)
                    curtime+=dur
            self.start = self.curve_pieces[0].start
            self.end = self.curve_pieces[-1].end

    def __str__(self):
        res = ["Trajectory. Start: {0}, End: {1}, Curves :".format(self.start,self.end)]
        for c in self.curve_pieces:
            res.append(str(c))
        res.append("----------------------------------------------------")
        return "\n".join(res)


    def evalfrom(self,time,granularity=100):
        if not self.start<=time<=self.end:
            raise ValueError("Time {0} is out of range. Curve start: {1}, Curve end: {2}".format(time,self.start,self.end))
        add = False
        res = []
        for c in self.curve_pieces:
            if not add:
                if c.start<=time<=c.end:
                    res.append(c.evalfrom(time,granularity=granularity))
                    add = True
            else:
                res.append(c.evalfrom(c.start,granularity=granularity))
        return np.column_stack(res)

    def to_marker(self,time,frame_id="map",granularity=100):
        #TODO: add coloring
        m = Marker()
        m.header.frame_id = frame_id
        m.type = m.LINE_STRIP
        m.color.a =1
        m.color.b = 1
        m.pose.orientation.w =1
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 0.05
        evarr = self.evalfrom(time,granularity=granularity)
        m.points= [creatept(x,y,z) for x,y,z in zip(evarr[0],evarr[1],evarr[2])]
        return m

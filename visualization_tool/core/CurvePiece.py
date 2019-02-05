import numpy as np
import bezier
"""
Piece object for curve. Each bezier curve consists of pieces. Each piece has a 
start time and end time. If no json has passed, an empty piece is formed.
@param json data is every element of trajectory key of the json
"""
class CurvePiece(object):

    def __init__(self,json=None):
        if json:
            self.start = float(json["min_parameter"])
            self.end = float(json["max_parameter"])
            if self.start>self.end:
                raise ValueError("Piece start must be less than or equal to piece end")
            self.curve = bezier.Curve.from_nodes(np.asfortranarray(json["controlpoints"]).T.astype('double'))
        else:
            self.start = None
            self.end = None
            self.curve = None

    def __str__(self):
        return "Piece, Start: {0}, End: {1}, Curve: {2}".format(self.start,self.end,self.curve)

    """
    Method returns curve points from time to end. Granularity is number of points to return for full range.
    For example if evaluating from the middle of the curve with 100 granularity, 50 points will be returned. 
    """
    def evalfrom(self,time,granularity=100):
        time = float(time)
        if not self.start<=time<=self.end: 
            raise ValueError("Time {0} is out of range. Curve start: {1}, Curve end: {2}".format(time,self.start,self.end))
        rng = self.end-self.start
        interval = self.end-time
        num_pts = (interval/rng)*granularity
        rng_start = 1-(interval/rng)
        return self.curve.evaluate_multi(np.linspace(rng_start,1,num_pts))
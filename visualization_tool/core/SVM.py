from core.Hyperplane import Hyperplane
from collections import defaultdict

class SVM(object):

    def __init__(self,hyperplanes_key):
        self.pieces = defaultdict(list)
        for el in hyperplanes_key:
            pid = el["piece_id"]
            nm = el["hyperplane"]["normal"]
            dst = el["hyperplane"]["distance"]
            self.pieces[pid].append(Hyperplane(nm,dst))
    
    def get_markers(self,maxPiece):
        res = []
        for x in range(maxPiece+1):
            res.extend([e.to_marker() for e in self.pieces[x]])
        return res
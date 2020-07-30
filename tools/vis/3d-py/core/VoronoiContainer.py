from core.Voronoi import Voronoi

class VoronoiContainer(object):

    def __init__(self,copy=None,numRobots=None):
        if copy:
            self.voronois = [x for x in copy.voronois]
        else:
            self.voronois = [None for x in xrange(numRobots)]
    
    def update(self,voronoi_hyperplanes_key):
        for el in voronoi_hyperplanes_key:
            rid = el["robot_id"]
            if "hyperplanes" in el:
                self.voronois[rid] = Voronoi(el["hyperplanes"])

    def get_all_markers(self):
        res = []
        for x in self.voronois:
            if x==None: continue
            res.extend(x.get_markers())
        return res

        
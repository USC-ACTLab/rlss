from Hyperplane import Hyperplane

class Voronoi(object):
    def __init__(self,hyperplanes_key):
        self.hyperplanes = []
        for el in hyperplanes_key:
            self.hyperplanes.append(Hyperplane(el["normal"],el["distance"]))
    
    def get_markers(self):
        return [x.to_marker() for x in self.hyperplanes]
        
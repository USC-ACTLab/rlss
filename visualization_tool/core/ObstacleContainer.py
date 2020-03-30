import json
from core.Obstacle import Obstacle

class ObstacleContainer(object):

    def __init__(self,copy=None):
        if copy:
            self.obstacles = copy.obstacles
        else:
            self.obstacles = []

    def update(self,obstacle_key):
        self.obstacles = [Obstacle(points=p) for p in obstacle_key]

    def get_all_markers(self):
        return [p.to_marker() for p in self.obstacles]

if __name__=="__main__":
    with open("../data/samplex.json",'r') as fl:
        js = json.load(fl)
        oc = ObstacleContainer(js["frames"][0]["obstacles"])
        print(oc.get_all_markers())
import json
from Trajectory import Trajectory
from DiscretePath import DiscretePath

class PathContainer(object):

    def __init__(self,json=[],numRobots=None,copy=None):
        if copy:
            self.trajectories = [x for x in copy.trajectories]
            self.discrete_paths = [x for x in copy.discrete_paths]
        else:
            if numRobots==None: raise ValueError("Cannot initialze Pathcontainer without numRobots")
            self.trajectories = [None for x in xrange(numRobots)]
            self.discrete_paths = [None for x in xrange(numRobots)]

    def update(self,trajectories_key):
        for el in trajectories_key:
            rid = el["robot_id"]
            self.trajectories[rid] = Trajectory(json_file=el["trajectory"])
            self.discrete_paths[rid] = DiscretePath(json=el["discrete_path"])

    def get_all_markers(self,evalfrom):
        res = []
        res.extend([x.to_marker(evalfrom,granularity=10) for x in self.trajectories if x!=None])
        res.extend([x.to_marker() for x in self.discrete_paths if x!=None])
        return res
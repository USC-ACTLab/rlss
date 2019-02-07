from Robot import Robot
import json

class RobotContainer(object):

    def __init__(self,numRobots=None,radius=None,copy=None):
        if copy:
            self.robots = [x for x in copy.robots]
            self.radius = copy.radius
        else:
            if numRobots==None or radius==None : raise ValueError("Robot container cannot initialized without radius and numrobots")
            self.robots = [None for x in xrange(numRobots)]
            self.radius = radius

    def update(self,robot_positions_key):
        for j in robot_positions_key:
            rid = j["robot_id"]
            px,py,pz= j["position"]
            self.robots[rid] = Robot(id=rid,x=px,y=py,z=pz,radius=self.radius)

    def get_all_markers(self):
        return [r.to_marker() for r in self.robots if r]



if __name__=="__main__":
    with open("../data/samplex.json",'r') as fl:
        js = json.load(fl)
        rb = RobotContainer(js["frames"][0]["robot_positions"],5,0.5)
        print rb.get_all_markers()
    
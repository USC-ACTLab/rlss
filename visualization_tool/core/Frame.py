from PathContainer import PathContainer
from ObstacleContainer import ObstacleContainer
from OccupiedCellsContainer import OccupiedCellsContainer
from RobotContainer import RobotContainer
from VoronoiContainer import VoronoiContainer
from SVMContainer import SVMContainer
from visualization_msgs.msg import MarkerArray
from rospy import Duration
"""
TODO:
    -write templates for Frame Container and frame
    -add default constructor and update function for every container
    -add colors
    -create class Frame(this)
    -create class FrameContainer
    -create class PathContainer,VoronoiContainer,SVMContainer
"""

class Frame(object):

    def __init__(self,numRobots=0,robot_radius=0.1,copy=None,step=None,frame_dt = None):
        #copy constructor copies all pointers from other frame in a new array
        if copy:
            self.path_container = PathContainer(copy=copy.path_container)
            self.obstacle_container = ObstacleContainer(copy=copy.obstacle_container)
            self.occupied_cells_container = OccupiedCellsContainer(copy=copy.occupied_cells_container)
            self.robot_container = RobotContainer(copy=copy.robot_container)
            self.voronoi_container = VoronoiContainer(copy=copy.voronoi_container)
            self.svm_container = SVMContainer(copy=copy.svm_container)
            self.frame_dt = copy.frame_dt
        else:
            #default constructor inits with nones
            #array of {robot_id,trajectory,discrete_path},size=nR ->PathContainer 
            self.path_container = PathContainer(numRobots=numRobots)
            #array of obstacle_container
            self.obstacle_container = ObstacleContainer()
            #array of occupied_cells_container
            self.occupied_cells_container = OccupiedCellsContainer()
            #array of robots, size=nR
            self.robot_container = RobotContainer(numRobots=numRobots,radius=robot_radius)
            #array of {robot_id,hyperplane[]} size=nR -> VoronoiContainer
            self.voronoi_container = VoronoiContainer(numRobots=numRobots)
            #array of {robot_id,{piece_id,hyperplane}[]},size nR -> SVMContainer
            #use maxpiece pieces of it
            self.svm_container = SVMContainer(numRobots=numRobots,config=None)
            self.frame_dt = frame_dt
    """
    If the frame has any information about new data, hyperplanes etc create new object
    and set the new pointer
    """
    def update(self,newFrame):
        self.step = newFrame["step"]
        if newFrame.has_key("trajectories"): self.path_container.update(newFrame["trajectories"])
        if newFrame.has_key("obstacles"):self.obstacle_container.update(newFrame["obstacles"])
        if newFrame.has_key("occupied_cells"):self.occupied_cells_container.update(newFrame["occupied_cells"])
        if newFrame.has_key("robot_positions"):self.robot_container.update(newFrame["robot_positions"])
        if newFrame.has_key("voronoi_hyperplanes"):self.voronoi_container.update(newFrame["voronoi_hyperplanes"])
        if newFrame.has_key("svm_hyperplanes"):self.svm_container.update(newFrame["svm_hyperplanes"])


    """
    this creates a marker array from Frame, for publishing
    """
    def toMarkerArray(self,timestamp=None):
        markers = []
        markers.extend(self.path_container.get_all_markers(float(self.frame_dt)*self.step))
        markers.extend(self.obstacle_container.get_all_markers())
        markers.extend(self.occupied_cells_container.get_all_markers())
        markers.extend(self.robot_container.get_all_markers())
        markers.extend(self.voronoi_container.get_all_markers())
        markers.extend(self.svm_container.get_all_markers())
        msg = MarkerArray()
        msg.markers = markers
        sq = 0
        for ms in markers:
            if timestamp!=None : ms.header.stamp = timestamp
            ms.lifetime = Duration(secs=self.frame_dt)
            ms.id = sq 
            sq+=1
        return msg

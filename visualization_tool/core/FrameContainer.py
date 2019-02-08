
from Frame import Frame
import json

"""
Container class for frames.
"""
class FrameContainer(object):

    def __init__(self,json_path):
        self.frames = []
        self.times = []
        with open(json_path) as fl:
            js = json.load(fl)
        rc = js["robot_count"]
        frame_dt = js["frame_dt"]
        frames = js["frames"]
        old_frame = Frame(numRobots=rc,robot_radius=js["robot_radius"],frame_dt=frame_dt) #create empty frame for size rc robots
        for f in frames:
            new_frame = Frame(copy=old_frame) #set new frame as copy of old
            new_frame.update(f) #update new frame with json
            self.frames.append(new_frame) #append updated frame to self
            self.times.append(float(frame_dt)*float(new_frame.step))
            old_frame = new_frame #set new frame as old frame

    def getFrame(self,frame_id):
        #returns frames and times for this container
        return self.frames[frame_id],self.times[frame_id]

if __name__=="__main__":
    fc = FrameContainer("/home/alpcevikel/mr-trajectory-replanning/visualization_tool/data/samplex.json")
    fr,tm = fc.getFrame(0)
    print fr.toMarkerArray()

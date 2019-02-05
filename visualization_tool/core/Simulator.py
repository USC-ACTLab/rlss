from time import time,sleep
import rospy
import sys, select, termios, tty
import select
from bisect import bisect_right


class Simulator (object):

    def __init__(self):
        self.times = [1,5,15,20,25,35]
        self.data = ["A","B","C","D","E","F"]
        #self.data = [(10,"A"),(20,"B")]
        self.cur_frame = None
        self.read_list = [sys.stdin]
        self.timeout = 0.1
        self.last_work_time = time()
        self.rate = 2.0
        self.play = False

        
    def setTime(self,time):
        target_time = max(0,self.ct+time)
        target_ind = bisect_right(self.times,target_time)-1
        target_ind = max(0,target_ind)
        target_ind = min(len(self.data)-1,target_ind)
        self.ct = target_time
        self.cur_frame = -1
        self.frame = target_ind
        print "Time has been set to: {0}, with index {1}".format(target_time,target_ind)
    
    def treat_input(self,line):
        line = line.strip()
        if line=='p':
            self.play = not self.play
            if self.play:
                print "PLAYING"
            else: 
                print "PAUSED"
        elif ' ' in line and 't' in line:
            p1,p2 = line.split(' ')
            try:
                if p1=="t":
                    p2 = float(p2)
                    self.setTime(p2)
                else:
                    print "Input not found"
            except:
                print "Input not float"
        else:
            print  "Input not found"
        self.last_work_time = time()

    def update_frame(self):
        now = time()
        if now-self.last_work_time>1/self.rate:
            td = time()-self.lastTime
            self.lastTime = time()
            if self.play: self.ct+=td
            while self.times[self.frame]<self.ct:
                self.frame+=1
                if self.frame>=len(self.data): break
            self.setFrame(self.frame)
            self.last_work_time = now
    
    def main_loop(self):
        self.ct = 0
        self.frame = 0
        self.lastTime = time()
        while self.read_list and self.ct<self.times[-1]:
            print "Current time: ",self.ct
            ready = select.select(self.read_list, [], [], self.timeout)[0]
            if not ready:
                self.update_frame()
            else:
                for file in ready:
                    line = file.readline()
                    if not line: # EOF, remove file from input list
                        self.read_list.remove(file)
                    elif line.rstrip(): # optional: skipping empty lines
                        self.treat_input(line)       

    def setFrame(self,frame):
        if self.cur_frame == frame : return
        if frame>=len(self.data): return
        #handle frame here
        print self.data[frame]," ",self.ct
        #handle end  
        self.cur_frame = frame
    
    def run(self):
        try: 
            self.main_loop()
        except KeyboardInterrupt:
            pass
            
            

if __name__== "__main__":
    sm = Simulator()
    sm.run()

from visualizer.Visualizer import Visualizer
from cmd import Cmd

import readline
readline.set_completer_delims(' \t\n')
import os.path as op
import glob as gb
import json
import threading
import rospy

def run_visualizer(vis):
    vis.run()

class VisualizerPrompt(Cmd):
    prompt = '3dvis> '
    intro = 'Type ? for commands'

    def complete_start(self, text, line, begidx, endidx):
        return self._complete_path(text)

    def do_start(self, inp):
        try:
            if self.visualizer_thread != None and self.visualizer_thread.is_alive():
                self.command_queue.put({
                    "action": "exit"
                })
                self.visualizer_thread.join()
        except AttributeError:
            pass

        f = open(inp, "r")
        j = json.loads(f.read())
        f.close()

        self.vis = Visualizer(j)
        self.command_queue = self.vis.command_queue

        self.visualizer_thread = self.auto_builder_thread = threading.Thread(target = run_visualizer, args=(self.vis,))
        self.visualizer_thread.start()

    def do_stop(self, inp):
        if self.visualizer_thread != None and self.visualizer_thread.is_alive():
            self.command_queue.put({
                "action": "exit"
            })
            self.visualizer_thread.join()

    def do_pause(self, inp):
        if self.visualizer_thread != None and self.visualizer_thread.is_alive():
            self.command_queue.put({
                "action": "pause"
            })

    def do_unpause(self, inp):
        if self.visualizer_thread != None and self.visualizer_thread.is_alive():
            self.command_queue.put({
                "action": "unpause"
            })


    def do_forward(self, inp):
        if self.visualizer_thread != None and self.visualizer_thread.is_alive():
            self.command_queue.put({
                "action": "forward"
            })


    def do_backward(self, inp):
        if self.visualizer_thread != None and self.visualizer_thread.is_alive():
            self.command_queue.put({
                "action": "backward"
            })

    def do_step(self, inp):
        if self.visualizer_thread != None and self.visualizer_thread.is_alive():
            if inp.strip() == "":
                cnt = 1
            else:
                try:
                    cnt = int(inp)
                except:
                    cnt = 1

            self.command_queue.put({
                "action": "step",
                "count": cnt
            })

    def do_exit(self, inp):
        self.do_stop(inp)
        print("bye")
        return True

    def _complete_path(self, path):
        if op.isdir(path):
            return gb.glob(op.join(path, '*'))
        else:
            return gb.glob(path+'*')


rospy.init_node("rlss_3d_visualizer")
VisualizerPrompt().cmdloop()

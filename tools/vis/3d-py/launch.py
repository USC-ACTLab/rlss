from core.Simulator import Simulator
import sys

if(len(sys.argv)<2):
    exit("Usage: frame_test.py <json-path>")

jsn_path = sys.argv[1]

if __name__== "__main__":
    sm = Simulator(jsn_path)
    sm.run()

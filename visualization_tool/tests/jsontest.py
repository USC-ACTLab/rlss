import json
import bezier
import numpy as np 
import csv
from Trajectory import Trajectory


if __name__ == "__main__":
    fl = open("sample.json",'r')
    t = json.load(fl)
    x= t["original_trajectories"][0]
    pieces = x["trajectory"]
    tre = Trajectory(csv_file_2d="example2d.csv")
    tre2 = Trajectory(json_file=x["trajectory"])
    tre.to_marker(90)
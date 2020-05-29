import csv
import sys
import json

p = sys.argv[1]
csvfile = open(p, "r")
csvreader = csv.reader(csvfile, delimiter=',')
next(csvreader)

pieces = []

for piece_row in csvreader:
    piece = {
        "type": "BEZIER",
        "control_points": []
    }
    piece["duration"]= float(piece_row[0])
    for i in range(1, len(piece_row), 3):
        piece["control_points"].append([float(piece_row[i]), float(piece_row[i+1]), float(piece_row[i+2])])

    pieces.append(piece)

print(json.dumps(pieces))
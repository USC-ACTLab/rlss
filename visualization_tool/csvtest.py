import csv
import numpy as np 
import bezier
with open('example2d.csv', 'rb') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     first = True
     starttime = 0 
     for row in spamreader:
        if first:
            first = False
            continue
        dur = row[0]
        cpts = row[1:]
        curpiece = []
        for i,k in zip(cpts[0::2], cpts[1::2]):
            curpiece.append([float(i),float(k),float(0)])
        curve =  bezier.Curve.from_nodes(np.asfortranarray(curpiece).T.astype("double"))
        print curve
        print "------------------------------------------"
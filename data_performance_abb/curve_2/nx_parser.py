import numpy as np
from pandas import *

fname="leading_edge2+0.02"

f = open(fname+".txt", "r")
curve=[]
for line in f.readlines()[3:]:
	curve.append(list(map(float,line.split())))
#convert to mm from inch
curve=25.4*np.array(curve)
DataFrame(curve).to_csv(fname+'.csv',header=False,index=False)

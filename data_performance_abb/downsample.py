from pandas import *
import numpy as np

dataset='wood/'
curve = read_csv(dataset+"Curve_dense.csv",header=None).values

curve_downsampled=np.vstack((curve[::500],curve[-1]))
print(len(curve_downsampled))
DataFrame(curve_downsampled).to_csv(dataset+'Curve_sparse.csv',header=False,index=False)
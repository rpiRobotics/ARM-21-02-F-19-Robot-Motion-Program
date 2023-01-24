
import numpy as np
from pandas import *
import sys
from general_robotics_toolbox import *


col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z'] 
data = read_csv("from_ge/Curve_in_base_frame2.csv", names=col_names)

curve_direction_x=data['direction_x'].tolist()
curve_direction_y=data['direction_y'].tolist()
curve_direction_z=data['direction_z'].tolist()

curve_direction=np.vstack((curve_direction_x, curve_direction_y, curve_direction_z))

data = read_csv("from_ge/Curve_dense_new_mm.csv", names=col_names)
curve_x=data['X'].tolist()
curve_y=data['Y'].tolist()
curve_z=data['Z'].tolist()
curve=np.vstack((curve_x, curve_y, curve_z)).T


###reference frame transformation
R=np.array([[0,0,1.],
			[1.,0,0],
			[0,1.,0]])

#convert curve direction to base frame
curve_direction=np.dot(R.T,curve_direction).T


df=DataFrame({'x':curve[:,0],'y':curve[:,1], 'z':curve[:,2],'x_direction':curve_direction[:,0],'y_direction':curve_direction[:,1],'z_direction':curve_direction[:,2]})
df.to_csv('from_ge/relative_path.csv',header=False,index=False)
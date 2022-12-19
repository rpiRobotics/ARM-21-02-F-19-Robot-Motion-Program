
import numpy as np
from pandas import *
import sys
from general_robotics_toolbox import *
sys.path.append('../toolbox')
from robots_def import *

data_dir='wood/'

col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z'] 
data = read_csv(data_dir+"Curve_dense.csv", names=col_names)
curve_x=data['X'].tolist()
curve_y=data['Y'].tolist()
curve_z=data['Z'].tolist()
curve_direction_x=data['direction_x'].tolist()
curve_direction_y=data['direction_y'].tolist()
curve_direction_z=data['direction_z'].tolist()

curve=np.vstack((curve_x, curve_y, curve_z)).T
curve_direction=np.vstack((curve_direction_x, curve_direction_y, curve_direction_z))

###reference frame transformation, from cad frame to tool frame, align z axis
# R=np.eye(3)
# T=np.zeros((3,1))

R=np.dot(Rz(np.radians(90)),Rx(np.radians(180)))
T=np.array([[-300.],[0.],[462.]])			###offset in blade frame 
H=np.vstack((np.hstack((R,T)),np.array([0,0,0,1])))

###attach to tool frame
curve_tool=curve.T+np.tile(T,(len(curve)))
curve_tool=np.dot(R,curve_tool).T
curve_direction=np.dot(R,curve_direction).T

curve_tool=np.flip(curve_tool,axis=0)
curve_direction=np.flip(curve_direction,axis=0)
df=DataFrame({'x':curve_tool[:,0],'y':curve_tool[:,1], 'z':curve_tool[:,2],'x_direction':curve_direction[:,0],'y_direction':curve_direction[:,1],'z_direction':curve_direction[:,2]})
df.to_csv(data_dir+'relative_path_tool_frame.csv',header=False,index=False)
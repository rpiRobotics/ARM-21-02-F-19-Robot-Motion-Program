
import numpy as np
from pandas import *
import sys
from general_robotics_toolbox import *
sys.path.append('../toolbox')
from robots_def import *

data_dir='wood/'
curve_pose_dir='curve_pose_opt3/'

col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z'] 
data = read_csv(data_dir+"Curve_dense.csv", names=col_names)
curve_x=data['X'].tolist()
curve_y=data['Y'].tolist()
curve_z=data['Z'].tolist()
curve_direction_x=data['direction_x'].tolist()
curve_direction_y=data['direction_y'].tolist()
curve_direction_z=data['direction_z'].tolist()

curve=np.vstack((curve_x, curve_y, curve_z)).T
curve_normal=np.vstack((curve_direction_x, curve_direction_y, curve_direction_z)).T

###reference frame transformation
with open(data_dir+curve_pose_dir+'blade_pose.yaml') as file:
	curve_pose = np.array(yaml.safe_load(file)['H'],dtype=np.float64)


R_curve=curve_pose[:3,:3]
shift=curve_pose[:-1,-1]

curve_new=np.dot(R_curve,curve.T).T+np.tile(shift,(len(curve),1))
curve_normal_new=np.dot(R_curve,curve_normal.T).T


df=DataFrame({'x':curve_new[:,0],'y':curve_new[:,1], 'z':curve_new[:,2],'x_direction':curve_normal_new[:,0],'y_direction':curve_normal_new[:,1],'z_direction':curve_normal_new[:,2]})
df.to_csv(data_dir+curve_pose_dir+'Curve_in_base_frame.csv',header=False,index=False)
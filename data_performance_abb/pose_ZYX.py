
import numpy as np
from pandas import *
import sys
from general_robotics_toolbox import *
from utils import *
from robots_def import *


# dataset='curve_2/'
# solution_dir='curve_pose_opt2/'
# # solution_dir='curve_pose_opt1/'
# data_dir=dataset+solution_dir

# ###reference frame transformation
# curve_pose=np.loadtxt(data_dir+'curve_pose.csv',delimiter=',')

# print(curve_pose[:-1,-1])
# print(np.degrees(rotationMatrixToEulerAngles(curve_pose[:3,:3])))


dataset='curve_2/'
solution_dir='dual_arm/diffevo_pose6_3/'
data_dir=dataset+solution_dir

###reference frame transformation
curve_pose=np.loadtxt(data_dir+'base.csv',delimiter=',')

# curve_pose=H_from_RT(curve_pose[:-1,:-1].T,-curve_pose[:-1,:-1].T@curve_pose[:-1,-1])

# print(curve_pose)
print(curve_pose[:-1,-1])
print(np.degrees(rotationMatrixToEulerAngles(curve_pose[:3,:3])))


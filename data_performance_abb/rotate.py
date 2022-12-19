import sys, yaml, copy
import numpy as np
from general_robotics_toolbox import *
from pandas import read_csv, DataFrame

data_dir='curve_2/'
original_dir=data_dir+'curve_pose_opt2/'
curve_pose = np.loadtxt(original_dir+'curve_pose.csv',delimiter=',')
curve_js = read_csv(original_dir+'Curve_js.csv',header=None).values
curve_dense = read_csv(data_dir+'Curve_dense.csv',header=None).values


rotate_angle=np.pi/8

H_R=np.eye(4)
H_R[:3,:3]=rot([0,0,1],rotate_angle)
curve_pose_new=H_R@curve_pose

new_dir=original_dir[:-1]+'_2/'
np.savetxt(new_dir+'curve_pose.csv',curve_pose_new,delimiter=',')

curve_js_new=copy.deepcopy(curve_js)
curve_js_new[:,0]=curve_js[:,0]+rotate_angle


curve_new=np.dot(curve_pose_new[:3,:3],curve_dense[:,:3].T).T+np.tile(curve_pose_new[:-1,-1],(len(curve_dense),1))
curve_normal_new=np.dot(curve_pose_new[:3,:3],curve_dense[:,3:].T).T


df=DataFrame({'q0':curve_js_new[:,0],'q1':curve_js_new[:,1],'q2':curve_js_new[:,2],'q3':curve_js_new[:,3],'q4':curve_js_new[:,4],'q5':curve_js_new[:,5]})
df.to_csv(new_dir+'Curve_js.csv',header=False,index=False)
df=DataFrame({'x':curve_new[:,0],'y':curve_new[:,1],'z':curve_new[:,2],'nx':curve_normal_new[:,0],'ny':curve_normal_new[:,1],'nz':curve_normal_new[:,2]})
df.to_csv(new_dir+'Curve_in_base_frame.csv',header=False,index=False)
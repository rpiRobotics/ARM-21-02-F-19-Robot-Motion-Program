import sys, yaml, copy
import numpy as np
from general_robotics_toolbox import *
from pandas import read_csv, DataFrame

original_dir='curve_2/dual_arm/diffevo_pose6_2/'

abb1200_pose = np.loadtxt(original_dir+'base.csv',delimiter=',')
curve_js = read_csv(original_dir+'arm1.csv',header=None).values


rotate_angle=np.pi/6

H_R=np.eye(4)
H_R[:3,:3]=rot([0,0,1],rotate_angle)
abb1200_pose_new=H_R@abb1200_pose

# new_dir=original_dir#[:-1]+'_2/'
new_dir='curve_2/dual_arm/diffevo_pose6_3/'
np.savetxt(new_dir+'base.csv',abb1200_pose_new,delimiter=',')

curve_js_new=copy.deepcopy(curve_js)
curve_js_new[:,0]=curve_js[:,0]+rotate_angle


df=DataFrame({'q0':curve_js_new[:,0],'q1':curve_js_new[:,1],'q2':curve_js_new[:,2],'q3':curve_js_new[:,3],'q4':curve_js_new[:,4],'q5':curve_js_new[:,5]})
df.to_csv(new_dir+'arm1.csv',header=False,index=False)

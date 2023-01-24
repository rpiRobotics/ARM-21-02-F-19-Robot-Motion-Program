import numpy as np 
import copy 
from utils import *
from robots_def import *
from dual_arm import *

height_offset=25		##24plate-4carpet

solution_dir='../diffevo_pose6/'

base=np.loadtxt(solution_dir+'base.csv',delimiter=',')

base_new=copy.deepcopy(base)
base_new[2,-1]-=height_offset

np.savetxt('base.csv',base_new,delimiter=',')

robot1=robot_obj('ABB_6640_180_255','../../../../config/abb_6640_180_255_robot_default_config.yml',tool_file_path='../../../../config/paintgun.csv',d=50,acc_dict_path='')
robot2_new=robot_obj('ABB_1200_5_90','../../../../config/abb_1200_5_90_robot_default_config.yml',tool_file_path=solution_dir+'tcp.csv',base_transformation_file='base.csv',acc_dict_path='')
robot2=robot_obj('ABB_1200_5_90','../../../../config/abb_1200_5_90_robot_default_config.yml',tool_file_path=solution_dir+'tcp.csv',base_transformation_file=solution_dir+'base.csv',acc_dict_path='')

curve_js1=np.loadtxt(solution_dir+'arm1.csv',delimiter=',')
curve_js2=np.loadtxt(solution_dir+'arm2.csv',delimiter=',')
pose_all=robot1.fwd(curve_js1)

p_all_new=copy.deepcopy(pose_all.p_all)
p_all_new[:,-1]-=height_offset

curve_js1_new=car2js(robot1,curve_js1[0],p_all_new,pose_all.R_all)

np.savetxt('arm1.csv',curve_js1_new,delimiter=',')

_,_,_,_,relative_path,relative_path_R=form_relative_path(curve_js1_new,curve_js2,robot1,robot2_new)

print(relative_path)

import numpy as np
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from robots_def import *
from utils import *
from lambda_calc import *

from lambda_calc import *
from utils import *
from dual_arm import *

dataset='curve_2/'
data_dir=dataset
solution_dir=data_dir+'dual_arm/'+'diffevo_pose6/'
cmd_dir=solution_dir+'30L_relative/'
num_ls=[30]

robot1=robot_obj('ABB_6640_180_255','../config/abb_6640_180_255_robot_default_config.yml',tool_file_path='../config/paintgun.csv',d=50,acc_dict_path='')
robot2=robot_obj('ABB_1200_5_90','../config/abb_1200_5_90_robot_default_config.yml',tool_file_path=solution_dir+'tcp.csv',base_transformation_file=solution_dir+'base.csv',acc_dict_path='')

relative_path,lam_relative_path,lam1,lam2,curve_js1,curve_js2=initialize_data(dataset,data_dir,solution_dir,robot1,robot2)

curve2 = []
for i in range(len(curve_js1)):
	curve2.append(robot2.fwd(curve_js2[i]).p)
curve2=np.array(curve2)


for num_l in num_ls:
	breakpoints=np.linspace(0,len(curve_js1),num_l+1).astype(int)
	points1=[]
	points1.append([np.array(relative_path[0,:3])])
	points2=[]
	points2.append([np.array(curve2[0,:3])])

	q_bp1=[]
	q_bp1.append([np.array(curve_js1[0])])
	q_bp2=[]
	q_bp2.append([np.array(curve_js2[0])])
	for i in range(1,num_l+1):
		points1.append([np.array(relative_path[breakpoints[i]-1,:3])])
		q_bp1.append([np.array(curve_js1[breakpoints[i]-1])])
		
		points2.append([np.array(curve2[breakpoints[i]-1,:3])])
		q_bp2.append([np.array(curve_js2[breakpoints[i]-1])])


	breakpoints[1:]=breakpoints[1:]-1

	primitives_choices1=['moveabsj']+['movel']*num_l
	df=DataFrame({'breakpoints':breakpoints,'primitives':primitives_choices1,'p_bp':points1,'q_bp':q_bp1})
	df.to_csv(cmd_dir+'command1.csv',header=True,index=False)

	primitives_choices2=['moveabsj']+['moveabsj']*num_l
	df=DataFrame({'breakpoints':breakpoints,'primitives':primitives_choices2,'p_bp':points2,'q_bp':q_bp2})
	df.to_csv(cmd_dir+'command2.csv',header=True,index=False)

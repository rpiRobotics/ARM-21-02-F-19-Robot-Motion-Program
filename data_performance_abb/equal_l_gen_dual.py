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


data_dir='curve_2/dual_arm/'
solution_dir=data_dir+'diffevo_pose6_3/'
cmd_dir=solution_dir+'35L/'
num_ls=[35]

robot=robot_obj('ABB_6640_180_255','../config/abb_6640_180_255_robot_default_config.yml',tool_file_path='../config/paintgun.csv',d=50,acc_dict_path='')
curve_js = read_csv(solution_dir+'arm1.csv',header=None).values


curve = []
for q in curve_js:
	curve.append(robot.fwd(q).p)
curve=np.array(curve)


curve_fit=np.zeros((len(curve_js),3))
curve_fit_R=np.zeros((len(curve_js),3,3))
for num_l in num_ls:
	breakpoints=np.linspace(0,len(curve_js),num_l+1).astype(int)
	points=[]
	points.append([np.array(curve[0,:3])])
	q_bp=[]
	q_bp.append([np.array(curve_js[0])])
	for i in range(1,num_l+1):
		points.append([np.array(curve[breakpoints[i]-1,:3])])
		q_bp.append([np.array(curve_js[breakpoints[i]-1])])

		if i==1:
			curve_fit[breakpoints[i-1]:breakpoints[i]]=np.linspace(curve[breakpoints[i-1],:3],curve[breakpoints[i]-1,:3],num=breakpoints[i]-breakpoints[i-1])
			R_init=robot.fwd(curve_js[breakpoints[i-1]]).R
			R_end=robot.fwd(curve_js[breakpoints[i]-1]).R
			curve_fit_R[breakpoints[i-1]:breakpoints[i]]=orientation_interp(R_init,R_end,breakpoints[i]-breakpoints[i-1])

		else:
			curve_fit[breakpoints[i-1]:breakpoints[i]]=np.linspace(curve[breakpoints[i-1]-1,:3],curve[breakpoints[i]-1,:3],num=breakpoints[i]-breakpoints[i-1]+1)[1:]
			R_init=robot.fwd(curve_js[breakpoints[i-1]-1]).R
			R_end=robot.fwd(curve_js[breakpoints[i]-1]).R
			curve_fit_R[breakpoints[i-1]:breakpoints[i]]=orientation_interp(R_init,R_end,breakpoints[i]-breakpoints[i-1]+1)[1:]
		
	primitives_choices=['moveabsj']+['movel']*num_l
	breakpoints[1:]=breakpoints[1:]-1
	df=DataFrame({'breakpoints':breakpoints,'primitives':primitives_choices,'p_bp':points,'q_bp':q_bp})
	df.to_csv(cmd_dir+'command1.csv',header=True,index=False)

	# curve_fit_js=car2js(robot,curve_js[0],curve_fit,curve_fit_R)
	# DataFrame(curve_fit_js).to_csv(data_dir+'curve_fit_js1.csv',header=False,index=False)
	# DataFrame(curve_fit).to_csv(data_dir+'curve_fit1.csv',header=False,index=False)



robot=robot_obj('ABB_1200_5_90','../config/abb_1200_5_90_robot_default_config.yml',tool_file_path=solution_dir+'tcp.csv',base_transformation_file=solution_dir+'base.csv',acc_dict_path='')


curve_js = read_csv(solution_dir+'arm2.csv',header=None).values

curve = []
for q in curve_js:
	curve.append(robot.fwd(q).p)
curve=np.array(curve)


curve_fit=np.zeros((len(curve_js),3))
curve_fit_R=np.zeros((len(curve_js),3,3))
for num_l in num_ls:
	breakpoints=np.linspace(0,len(curve_js),num_l+1).astype(int)
	points=[]
	points.append([np.array(curve[0,:3])])
	q_bp=[]
	q_bp.append([np.array(curve_js[0])])
	for i in range(1,num_l+1):
		points.append([np.array(curve[breakpoints[i]-1,:3])])
		q_bp.append([np.array(curve_js[breakpoints[i]-1])])

		if i==1:
			curve_fit[breakpoints[i-1]:breakpoints[i]]=np.linspace(curve[breakpoints[i-1],:3],curve[breakpoints[i]-1,:3],num=breakpoints[i]-breakpoints[i-1])
			R_init=robot.fwd(curve_js[breakpoints[i-1]]).R
			R_end=robot.fwd(curve_js[breakpoints[i]-1]).R
			curve_fit_R[breakpoints[i-1]:breakpoints[i]]=orientation_interp(R_init,R_end,breakpoints[i]-breakpoints[i-1])

		else:
			curve_fit[breakpoints[i-1]:breakpoints[i]]=np.linspace(curve[breakpoints[i-1]-1,:3],curve[breakpoints[i]-1,:3],num=breakpoints[i]-breakpoints[i-1]+1)[1:]
			R_init=robot.fwd(curve_js[breakpoints[i-1]-1]).R
			R_end=robot.fwd(curve_js[breakpoints[i]-1]).R
			curve_fit_R[breakpoints[i-1]:breakpoints[i]]=orientation_interp(R_init,R_end,breakpoints[i]-breakpoints[i-1]+1)[1:]
		
	primitives_choices=['moveabsj']+['movel']*num_l
	breakpoints[1:]=breakpoints[1:]-1
	df=DataFrame({'breakpoints':breakpoints,'primitives':primitives_choices,'p_bp':points,'q_bp':q_bp})
	df.to_csv(cmd_dir+'command2.csv',header=True,index=False)

	# curve_fit_js=car2js(robot,curve_js[0],curve_fit,curve_fit_R)
	# DataFrame(curve_fit_js).to_csv(data_dir+'curve_fit_js2.csv',header=False,index=False)
	# DataFrame(curve_fit).to_csv(data_dir+'curve_fit2.csv',header=False,index=False)
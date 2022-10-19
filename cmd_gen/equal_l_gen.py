import numpy as np
from pandas import *
import sys, traceback, os
from robots_def import *
from utils import *

#input argument
robot_name='abb_6640'
dataset='curve_6/'
solution_dir=dataset+robot_name+'/baseline/'
num_l=100
cmd_dir=solution_dir+str(num_l)+'L/'

if not os.path.exists(cmd_dir):
	os.mkdir(cmd_dir)

robot=abb6640(d=50)

curve_js = np.loadtxt(solution_dir+'Curve_js.csv',delimiter=',')
curve = np.loadtxt(solution_dir+"Curve_in_base_frame.csv",delimiter=',')

curve_fit=np.zeros((len(curve_js),3))
curve_fit_R=np.zeros((len(curve_js),3,3))


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
	
primitives_choices=['moveabsj']+['movel_fit']*num_l

breakpoints[1:]=breakpoints[1:]-1
df=DataFrame({'breakpoints':breakpoints,'primitives':primitives_choices,'p_bp':points,'q_bp':q_bp})
df.to_csv(cmd_dir+'command.csv',header=True,index=False)

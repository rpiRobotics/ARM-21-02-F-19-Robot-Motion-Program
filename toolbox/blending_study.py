from pandas import read_csv, DataFrame
import sys, copy
sys.path.append('../circular_Fit')
from toolbox_circular_fit import *
from abb_motion_program_exec_client import *
from robots_def import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.interpolate import UnivariateSpline
from lambda_calc import *
from blending import *

def main():

	data_dir='../simulation/robotstudio_sim/scripts/fitting_output_new/threshold0.5/'

	data = read_csv(data_dir+'command.csv')
	breakpoints=np.array(data['breakpoints'].tolist())#[:3]
	primitives=data['primitives'].tolist()[1:]#[:2]

	col_names=['J1', 'J2','J3', 'J4', 'J5', 'J6'] 
	data=read_csv(data_dir+'curve_fit_js.csv',names=col_names)
	q1=data['J1'].tolist()
	q2=data['J2'].tolist()
	q3=data['J3'].tolist()
	q4=data['J4'].tolist()
	q5=data['J5'].tolist()
	q6=data['J6'].tolist()
	curve_js=np.vstack((q1,q2,q3,q4,q5,q6)).T.astype(float)


	speed='vmax'
	zone='z10'
	###read in curve_exe
	col_names=['timestamp', 'cmd_num', 'J1', 'J2','J3', 'J4', 'J5', 'J6'] 
	data = read_csv(data_dir+"curve_exe_"+speed+'_'+zone+".csv",names=col_names)
	q1=data['J1'].tolist()[1:]
	q2=data['J2'].tolist()[1:]
	q3=data['J3'].tolist()[1:]
	q4=data['J4'].tolist()[1:]
	q5=data['J5'].tolist()[1:]
	q6=data['J6'].tolist()[1:]
	timestamp=np.array(data['timestamp'].tolist()[1:]).astype(float)
	cmd_num=np.array(data['cmd_num'].tolist()[1:]).astype(float)
	start_idx=np.where(cmd_num==3)[0][0]
	curve_exe_js=np.radians(np.vstack((q1,q2,q3,q4,q5,q6)).T.astype(float)[start_idx:])


	robot=abb6640(d=50)

	curve_fit=[]
	curve_fit_R=[]
	ktheta_fit=[]
	for i in range(len(curve_js)):
		pose_temp=robot.fwd(curve_js[i])
		curve_fit.append(pose_temp.p)
		curve_fit_R.append(pose_temp.R)
		k,theta=R2rot(np.dot(curve_fit_R[-1],curve_fit_R[0].T))
		if theta==0:
			ktheta_fit.append(np.zeros(3))
		else:
			ktheta_fit.append(k*theta)
	ktheta_fit=np.array(ktheta_fit)

	curve_fit=np.array(curve_fit)

	act_breakpoints=breakpoints
	act_breakpoints[1:]=act_breakpoints[1:]-1

	lam=calc_lam_cs(curve_fit)[:act_breakpoints[-1]]

	lam_blended,q_blended=blend_cs(curve_js[act_breakpoints],curve_fit,breakpoints,lam,primitives,robot)

	curve_blended=[]
	for i in range(len(q_blended)):
		curve_blended.append(robot.fwd(q_blended[i]).p)
	curve_blended=np.array(curve_blended)

	curve_act=[]
	curve_act_R=[]
	ktheta_act=[]
	for i in range(len(curve_exe_js)):
		pose_temp=robot.fwd(curve_exe_js[i])
		curve_act.append(pose_temp.p)
		curve_act_R.append(pose_temp.R)
		k,theta=R2rot(np.dot(curve_act_R[-1],curve_act_R[0].T))
		if theta==0:
			ktheta_act.append(np.zeros(3))
		else:
			ktheta_act.append(k*theta)

	curve_act=np.array(curve_act)
	ktheta_act=np.array(ktheta_act)


	###plot results
	fig = plt.figure(1)
	ax = plt.axes(projection='3d')
	# ax.plot3D(curve_fit[:act_breakpoints[-1],0], curve_fit[:act_breakpoints[-1],1], curve_fit[:act_breakpoints[-1],2], c='red')
	ax.plot3D(curve_blended[:,0], curve_blended[:,1], curve_blended[:,2], c='red')
	ax.plot3D(curve_act[:,0], curve_act[:,1], curve_act[:,2], c='green')
	ax.scatter(curve_fit[breakpoints,0], curve_fit[breakpoints,1], curve_fit[breakpoints,2])

	fig = plt.figure(2)
	ax = plt.axes(projection='3d')
	ax.plot3D(ktheta_act[:,0], ktheta_act[:,1], ktheta_act[:,2], c='green')
	ax.plot3D(ktheta_fit[:,0], ktheta_fit[:,1], ktheta_fit[:,2], c='red')
	plt.show()

if __name__ == '__main__':
	main()

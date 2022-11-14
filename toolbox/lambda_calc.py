import numpy as np
import scipy, copy, time
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from robots_def import *
from error_check import *
from utils import * 
from scipy.optimize import fminbound
from scipy.signal import find_peaks
# from dual_arm import *

def calc_curvature(curve):
	lam=calc_lam_cs(curve)
	dlam=np.gradient(lam)
	curve_tan=np.gradient(curve,axis=0)
	curve_tan_mag = np.linalg.norm(curve_tan,axis=1)
	curve_tan_unit = curve_tan / curve_tan_mag[:, np.newaxis]

	d_curve_tan_unit=np.gradient(curve_tan_unit,axis=0)
	curvature=np.linalg.norm(d_curve_tan_unit,axis=1)/dlam

	return curvature


def calc_lam_js(curve_js,robot):
	#calculate lambda from joints
	lam=[0]
	curve=[]
	for i in range(len(curve_js)):
		robot_pose=robot.fwd(curve_js[i])
		curve.append(robot_pose.p)
		if i>0:
			lam.append(lam[-1]+np.linalg.norm(curve[i]-curve[i-1]))
	return np.array(lam)

def calc_lam_js_2arm(curve_js1,curve_js2,robot1,robot2,base2_R,base2_p):
	curve=[]
	for i in range(len(curve_js1)):
		pose1_now=robot1.fwd(curve_js1[i])
		pose2_world_now=robot2.fwd(curve_js2[i],base2_R,base2_p)

		curve.append(pose2_world_now.R.T@(pose1_now.p-pose2_world_now.p))
	# visualize_curve(np.array(curve))
	lam=calc_lam_cs(curve)
	return np.array(lam)

def calc_lam_cs(curve):
	###find path length
	temp=np.diff(curve,axis=0)
	temp=np.linalg.norm(temp,axis=1)
	lam=np.insert(np.cumsum(temp),0,0)

	return lam

def calc_lamdot(curve_js,lam,robot,step):
	############find maximum lambda dot vs lambda
	###curve_js: curve expressed in joint space in radians
	###lam: discrete lambda (path length), same shape as curve_js, from 0 to 1
	###robot: joint velocity & acc limit 
	###step: step size used for dense curve

	# dlam_max1=[]
	# dlam_max2=[]

	# dqdlam=[]
	# d2qdlam2=[]

	# for i in range(0,len(lam)-step,step):
	# 	dq=curve_js[i+step]-curve_js[i]
	# 	dqdlam.append(dq/(lam[i+step]-lam[i]))

	# 	dlam_max1.append(np.min(np.divide(robot.joint_vel_limit,np.abs(dqdlam[-1]))))
	# 	if i>0:
	# 		d2qdlam2.append(2*(dqdlam[-1]-dqdlam[-2])/(lam[i+step]-lam[i-step]))
	# 		dlam_max2.append(np.sqrt(np.min(np.divide(robot.joint_acc_limit,np.abs(d2qdlam2[-1])))))

	# dlam_max_act=np.minimum(np.array(dlam_max1),np.insert(dlam_max2,0,99999))

	curve_js=curve_js[::step]
	lam=lam[::step]
	dq=np.gradient(curve_js,axis=0)
	dlam=np.gradient(lam)
	dqdlam=np.divide(dq.T,dlam).T

	d2qdlam2=np.divide(np.gradient(dqdlam,axis=0).T,dlam).T

	dlam_max1=np.min(np.divide(robot.joint_vel_limit,np.abs(dqdlam)),axis=1)
	dlam_max2=np.sqrt(np.min(np.divide(robot.joint_acc_limit,np.abs(d2qdlam2)),axis=1))

	dlam_max_act=np.minimum(dlam_max1,dlam_max2)

	return dlam_max_act
def calc_lamdot_2arm(curve_js,lam,robot1,robot2,step):
	############find maximum lambda dot vs lambda
	###curve_js: curve expressed in joint space in radians
	###lam: discrete lambda (path length), same shape as curve_js, from 0 to 1
	###robot: joint velocity & acc limit 
	###step: step size used for dense curve

	curve_js=curve_js[::step]
	lam=lam[::step]
	dq=np.gradient(curve_js,axis=0)
	dlam=np.gradient(lam)
	dqdlam=np.divide(dq.T,dlam).T

	d2qdlam2=np.divide(np.gradient(dqdlam,axis=0).T,dlam).T

	dlam_max1=np.min(np.divide(np.hstack((robot1.joint_vel_limit,robot2.joint_vel_limit)),np.abs(dqdlam)),axis=1)
	dlam_max2=np.sqrt(np.min(np.divide(np.hstack((robot1.joint_acc_limit,robot2.joint_acc_limit)),np.abs(d2qdlam2)),axis=1))

	dlam_max_act=np.minimum(dlam_max1,dlam_max2)

	return dlam_max_act

def est_lamdot(dqdlam_list,breakpoints,lam,spl_list,merged_idx,robot):
	############estimated lambdadot from arbitray blending
	###dqdlam_list: list of dqdlam in each segment
	###lam: discrete lambda (path length), same shape as curve_js
	###breakpoints: breakpoints
	###spl_list: spline coefficient at each breakpoint
	###merged_idx: merged breakpoitn idx when blending
	###robot: robot definitions

	dlam_max1=[]
	dlam_max2=[]
	for dqdlam_seg in dqdlam_list:
		dlam_max1.append(np.min(np.divide(robot.joint_vel_limit,np.abs(dqdlam_seg))))

	###loop all spline blending
	for i in range(len(spl_list)):
		###loop all merged blending breakpoint
		if len(merged_idx[i])>1:
			bp=int(np.average(merged_idx[i]))
		else:
			bp=merged_idx[i][0]
		dq2dlam2=[]
		###loop all joints
		for j in range(len(robot.joint_vel_limit)):
			dderivative=spl_list[i][j].derivative().derivative()
			sampling=np.linspace(lam[bp]-5,lam[bp]+5)
			dq2dlam2.append(np.max(dderivative(lam[bp])))

		###calc lamdot_min at breakpoints
		dlam_max2.append(np.sqrt(np.min(np.divide(robot.joint_acc_limit,np.abs(dq2dlam2)))))
	return dlam_max1, dlam_max2
def est_lamdot_min(dqdlam_list,breakpoints,lam,spl_list,merged_idx,robot):
	dlam_max1,dlam_max2=est_lamdot(dqdlam_list,breakpoints,lam,spl_list,merged_idx,robot)
	# print(dlam_max2)
	return min(np.min(dlam_max1),np.min(dlam_max2))

def calc_lamdot2(curve_js,lam,robot,step):
	############find maximum lambda dot vs lambda, with different downsampling strategy, require full dense curve
	###curve_js: curve expressed in joint space in radians
	###lam: discrete lambda (path length), same shape as curve_js, from 0 to 1
	###robot: joint velocity & acc limit 
	###step: step size used for dense curve

	dlam_max1=[]
	dlam_max2=[]

	dqdlam=[]
	d2qdlam2=[]

	for i in range(1,len(lam)-step,step):
		dq=curve_js[i+1]-curve_js[i]
		dqdlam=dq/(lam[i+1]-lam[i])
		dlam_max1.append(np.min(np.divide(robot.joint_vel_limit,np.abs(dqdlam[-1]))))

		d2qdlam2.append(2*(dqdlam[-1]-dqdlam[-2])/(lam[i+1]-lam[i-1]))

		dlam_max2.append(np.sqrt(np.min(np.divide(robot.joint_acc_limit,np.abs(d2qdlam2[-1])))))



	dlam_max_act=np.minimum(np.array(dlam_max1),np.array(dlam_max2))


	return dlam_max1,dlam_max2

def calc_lamdot_acc_constraints(curve_js,lam,joint_vel_limit,joint_acc_limit,breakpoints,step):
	############find maximum lambda dot vs lambda
	###curve_js: curve expressed in joint space in radians
	###lam: discrete lambda (path length), same shape as curve_js, from 0 to 1
	###joint_vel_limit: joint velocity limit in radians

	dlam_max=[]

	joint_vel_prev=np.zeros(6)
	num_steps=int(len(curve_js)/step)
	idx=[]
	for i in range(len(breakpoints)-1):
		for j in range(breakpoints[i],1+breakpoints[i]+step*int((breakpoints[i+1]-breakpoints[i])/step),step):
			idx.append(j)

			next_step=min(breakpoints[i+1],j+step)

			dq=np.abs(curve_js[next_step]-curve_js[j])
			dqdlam=dq/(lam[next_step]-lam[j])
			t=np.max(np.divide(dq,joint_vel_limit))
			qdot_max=dq/t 		###approximated max qdot
			if j in breakpoints or next_step in breakpoints:
				t_acc=np.max(np.divide(joint_vel_prev,joint_acc_limit))
				if np.linalg.norm(dq-t_acc*qdot_max/2)>0:
					t_rem=np.max(np.divide(dq-t_acc*qdot_max/2,joint_vel_limit))
					qdot_act=dq/(t_acc+t_rem)
					dlam_max.append(qdot_act[0]/dqdlam[0])
					joint_vel_prev=qdot_max
				else:
					qddot_max=qdot_max/t_acc
					t_act=np.sqrt(2*dq/qddot_max)[0]
					dlam_max.append((qdot_max[0]/2)/dqdlam[0])
					joint_vel_prev=qddot_max*t_act
			else:
				dlam_max.append(qdot_max[0]/dqdlam[0])

	return dlam_max,idx


def calc_lamdot_dual(curve_js1,curve_js2,lam,joint_vel_limit1,joint_vel_limit2,step):
	############find maximum lambda dot vs lambda
	###curve_js1,2: curve expressed in joint space in radians for both robots
	###lam: discrete lambda (relative path length), same shape as curve_js, from 0 to 1
	###joint_vel_limit1,2: joint velocity limit in radians for both robots
	###step: step size used for dense curve

	dlam_max=[]

	for i in range(0,len(lam)-step,step):
		dq=np.abs(curve_js[i+step]-curve_js[i])
		dqdlam=dq/(lam[i+step]-lam[i])
		t=np.max(np.divide(dq,joint_vel_limit))

		qdot_max=dq/t 		###approximated max qdot
		dlam_max.append(qdot_max[0]/dqdlam[0])



	return dlam_max


def q_linesearch(alpha,qdot_prev,qdot_next,dt,joint_acc_limit):
	###equation: qdot1+qddot*dt/alpha=alpha*qdot2
	###alpha: coefficient of next qdot, (0,1]
	qddot=alpha*(alpha*qdot_next-qdot_prev)/dt
	coeff=np.abs(qddot)/joint_acc_limit

	###if find one alpha within acc constraint, take it
	if np.max(coeff)<1:
		return -alpha
	###else choose one alpha that brings outcome qddot closest to acc constraint
	else:
		return np.max(coeff)-1

def traj_speed_est(robot,curve_js,lam,vd,qdot_init=[]):
	####speed estimation given curve in joint space with qdot and qddot constraint
	#joint_acc_limit: joint acceleration at each pose of curve_js
	joint_acc_limit=robot.get_acc(curve_js)

	###find desired qdot at each step
	dq=np.gradient(curve_js,axis=0)
	dlam=np.gradient(lam)
	###replace 0 with small number
	dlam[dlam==0]=0.000001
	dt=dlam/vd
	qdot_d=np.divide(dq,np.tile(np.array([dt]).T,(1,6)))
	###bound desired qdot with qdot constraint
	qdot_max=np.tile(robot.joint_vel_limit,(len(curve_js),1))
	coeff=np.divide(np.abs(qdot_d),qdot_max)
	coeff=np.max(coeff,axis=1)	###get the limiting joint
	coeff=np.clip(coeff,1,999)	###clip coeff to 1
	coeff=np.tile(np.array([coeff]).T,(1,6))

	qdot=np.divide(qdot_d,coeff)	###clip propotionally

	
	#traversal
	if len(qdot_init)==0:
		qdot_act=[qdot[0]]
	else:
		qdot_act=[qdot_init]

	J0=robot.jacobian(curve_js[0])
	speed=[np.linalg.norm((J0@qdot_act[0])[3:])]

	###alpha: coeff of bounded qdot by qddot constraint
	alpha_all=[1]
	for i in range(1,len(curve_js)):
		qddot=(qdot[i]-qdot_act[-1])/dt[i]

		if np.any(np.abs(qddot)>joint_acc_limit[i]):
			alpha=fminbound(q_linesearch,0,1,args=(qdot_act[-1],qdot[i],dt[i],joint_acc_limit[i]))

		else:
			alpha=1
		qdot_act.append(alpha*qdot[i])
		alpha_all.append(alpha)

		J=robot.jacobian(curve_js[i])
		speed.append(np.linalg.norm((J@qdot_act[i])[3:]))

	speed=np.array(speed)
	qdot_act=np.array(qdot_act)

	alpha_all=np.array(alpha_all)
	alpha_jump_thresh=0.1
	alpha_smooth_thresh=0.01
	reverse=True
	iteration_num=0
	# plt.plot(speed)
	# plt.show()

	while np.max(np.abs(np.diff(alpha_all)))>alpha_jump_thresh:
		###identify large speed drop due to invalid acc from cur point
		large_drop_indices1,_=find_peaks(-speed)
		large_drop_indices2=np.squeeze(np.argwhere(np.abs(np.diff(alpha_all))>alpha_jump_thresh))
		large_drop_indices=np.intersect1d(large_drop_indices1,large_drop_indices2)
		if reverse:
			for large_drop_idx in large_drop_indices:
				###find closet point to start backtracking, without jump in alpha
				smooth_idx=np.argwhere(np.abs(np.diff(alpha_all))<alpha_smooth_thresh)
				temp_arr=smooth_idx-large_drop_idx
				back_trak_start_idx=min(len(curve_js)-2,smooth_idx[np.where(temp_arr > 0, temp_arr, np.inf).argmin()][0]+1) ###+/-1 here because used diff

				# print('backtrak')
				# print(back_trak_start_idx,large_drop_idx)
				###traverse backward
				for i in range(back_trak_start_idx,1,-1):

					qddot=(qdot_act[i+1]-qdot[i])/dt[i]

					if np.any(np.abs(qddot)>joint_acc_limit[i]):
						alpha=fminbound(q_linesearch,0,1,args=(qdot_act[i+1],qdot[i],dt[i],joint_acc_limit[i]))
					else:
						if i<large_drop_idx:
							reverse=False
							break
						alpha=1

					# print(i,alpha)
					qdot_act[i]=alpha*qdot[i]
					alpha_all[i]=alpha

					J=robot.jacobian(curve_js[i])
					speed[i]=np.linalg.norm((J@qdot_act[i])[3:])

		else:
			for large_drop_idx in large_drop_indices:
			# large_drop_idx=np.argwhere(np.abs(np.diff(alpha_all))>alpha_jump_thresh)[0][0]

				###find closet point to start forwardtracking, without jump in alpha
				smooth_idx=np.argwhere(np.abs(np.diff(alpha_all))<alpha_smooth_thresh)
				temp_arr=smooth_idx-large_drop_idx
				trak_start_idx=max(0,smooth_idx[np.where(temp_arr < 0, temp_arr, -np.inf).argmax()][0]-1)
				# print('forward')
				# print(trak_start_idx,large_drop_idx)
				###traverse backward
				for i in range(trak_start_idx,len(curve_js)):

					qddot=(qdot_act[i-1]-qdot[i])/dt[i]

					if np.any(np.abs(qddot)>joint_acc_limit[i]):
						alpha=fminbound(q_linesearch,0,1,args=(qdot_act[i-1],qdot[i],dt[i],joint_acc_limit[i]))
					else:
						if i>large_drop_idx:
							reverse=True
							break
						alpha=1
					# print(i,alpha)
					qdot_act[i]=alpha*qdot[i]
					alpha_all[i]=alpha

					J=robot.jacobian(curve_js[i])
					speed[i]=np.linalg.norm((J@qdot_act[i])[3:])

		# plt.plot(speed)
		# plt.show()

		iteration_num+=1
		if iteration_num>10:
			# print('exceed speed iteration')
			qdot_act=0.9*qdot_act
			if iteration_num>20:
				break
		# print(np.max(np.abs(np.diff(alpha_all))))


	###smooth out trajectory
	speed=replace_outliers2(speed,threshold=0.00001)
	speed=moving_average(speed,n=5,padding=True)
	return speed

def traj_speed_est_dual(robot1,robot2,curve_js1,curve_js2,base2_R,base2_p,lam,vd,qdot_init1=[],qdot_init2=[]):
	#############################TCP relative speed esitmation with dual arm##############################
	###lam & vd must both be 1. relative or 2. robot2's TCP
	joint_acc_limit1=robot1.get_acc(curve_js1)
	joint_acc_limit2=robot2.get_acc(curve_js2)
	###curve1_js & curve2_js must have same dimension
	curve_js=np.hstack((curve_js1,curve_js2))
	qdot_lim=np.hstack((robot1.joint_vel_limit,robot2.joint_vel_limit))
	qddot_lim=np.hstack((joint_acc_limit1,joint_acc_limit2))

	# ###form relative path
	# curve_relative=form_relative_path(robot1,robot2,curve_js1,curve_js2,base2_R,base2_p)

	###find desired qdot at each step
	dq=np.gradient(curve_js,axis=0)
	dlam=np.gradient(lam)
	dt=dlam/vd
	qdot_d=np.divide(dq,np.tile(np.array([dt]).T,(1,12)))
	###bound desired qdot with qdot constraint
	qdot_max=np.tile(qdot_lim,(len(curve_js),1))
	coeff=np.divide(np.abs(qdot_d),qdot_max)
	coeff=np.max(coeff,axis=1)	###get the limiting joint
	coeff=np.clip(coeff,1,999)	###clip coeff to 1
	coeff=np.tile(np.array([coeff]).T,(1,12))

	qdot=np.divide(qdot_d,coeff)	###clip propotionally

	#traversal
	if len(qdot_init1)==0:
		qdot_act=[qdot[0]]
	else:
		qdot_act=[np.hstack((qdot_init1,qdot_init2))]


	speed=[]
	speed1=[]
	speed2=[]
	alpha_all=[]

	for i in range(0,len(curve_js1)):
		pose1_now=robot1.fwd(curve_js1[i])
		pose2_now=robot2.fwd(curve_js2[i])
		pose2_world_now=robot2.fwd(curve_js2[i],base2_R,base2_p)

		J1=robot1.jacobian(curve_js1[i])
		J2=robot2.jacobian(curve_js2[i])

		J1p=np.dot(pose2_world_now.R.T,J1[3:,:])
		J2p=np.dot(pose2_now.R.T,J2[3:,:])
		J2R=np.dot(pose2_now.R.T,J2[:3,:])

		p12_2=pose2_world_now.R.T@(pose1_now.p-pose2_world_now.p)
		J_p=np.hstack((J1p,-J2p+hat(p12_2)@J2R))
		speed.append(np.linalg.norm(J_p@qdot_act[-1]))


		speed1.append(np.linalg.norm(J1p@qdot_act[-1][:6]))
		speed2.append(np.linalg.norm(J2p@qdot_act[-1][6:]))


		qddot=(qdot[i]-qdot_act[-1])/dt[i]

		if np.any(np.abs(qddot)>qddot_lim[i]):
			alpha=fminbound(q_linesearch,0,1,args=(qdot_act[-1],qdot[i],dt[i],qddot_lim[i]))
		else:
			alpha=1

		alpha_all.append(alpha)
		qdot_act.append(alpha*qdot[i])

	############################################forward/backward traversal##################################
	speed=np.array(speed)
	qdot_act=np.array(qdot_act)

	alpha_all=np.array(alpha_all)
	alpha_jump_thresh=0.1
	alpha_smooth_thresh=0.01
	reverse=True
	iteration_num=0
	# plt.plot(qdot_act[:,6])
	# plt.legend()
	# plt.show()

	while np.max(np.abs(np.diff(alpha_all)))>alpha_jump_thresh:
		###identify large speed drop due to invalid acc from cur point
		large_drop_indices1,_=find_peaks(-speed)
		large_drop_indices2=np.squeeze(np.argwhere(np.abs(np.diff(alpha_all))>alpha_jump_thresh))
		large_drop_indices=np.intersect1d(large_drop_indices1,large_drop_indices2)
		if reverse:
			for large_drop_idx in large_drop_indices:
				###find closet point to start backtracking, without jump in alpha
				smooth_idx=np.argwhere(np.abs(np.diff(alpha_all))<alpha_smooth_thresh)
				temp_arr=smooth_idx-large_drop_idx
				back_trak_start_idx=min(len(curve_js)-2,smooth_idx[np.where(temp_arr > 0, temp_arr, np.inf).argmin()][0]+1) ###+/-1 here because used diff

				# print('backtrak')
				# print(back_trak_start_idx,large_drop_idx)
				###traverse backward
				for i in range(back_trak_start_idx,1,-1):

					qddot=(qdot_act[i+1]-qdot[i])/dt[i]

					if np.any(np.abs(qddot)>qddot_lim):
						alpha=fminbound(q_linesearch,0,1,args=(qdot_act[i+1],qdot[i],dt[i],qddot_lim))
					else:
						if i<large_drop_idx:
							reverse=False
							break
						alpha=1

					# print(i,alpha)
					qdot_act[i]=alpha*qdot[i]
					alpha_all[i]=alpha
					####update speed
					pose1_now=robot1.fwd(curve_js1[i])
					pose2_now=robot2.fwd(curve_js2[i])
					pose2_world_now=robot2.fwd(curve_js2[i],base2_R,base2_p)

					J1=robot1.jacobian(curve_js1[i])
					J2=robot2.jacobian(curve_js2[i])

					J1p=np.dot(pose2_world_now.R.T,J1[3:,:])
					J2p=np.dot(pose2_now.R.T,J2[3:,:])
					J2R=np.dot(pose2_now.R.T,J2[:3,:])

					p12_2=pose2_world_now.R.T@(pose1_now.p-pose2_world_now.p)
					J_p=np.hstack((J1p,-J2p+hat(p12_2)@J2R))
					speed[i]=np.linalg.norm(J_p@qdot_act[i])


					speed1[i]=np.linalg.norm(J1p@qdot_act[i][:6])
					speed2[i]=np.linalg.norm(J2p@qdot_act[i][6:])


		else:
			for large_drop_idx in large_drop_indices:
			# large_drop_idx=np.argwhere(np.abs(np.diff(alpha_all))>alpha_jump_thresh)[0][0]

				###find closet point to start forwardtracking, without jump in alpha
				smooth_idx=np.argwhere(np.abs(np.diff(alpha_all))<alpha_smooth_thresh)
				temp_arr=smooth_idx-large_drop_idx
				trak_start_idx=max(0,smooth_idx[np.where(temp_arr < 0, temp_arr, -np.inf).argmax()][0]-1)
				# print('forward')
				# print(trak_start_idx,large_drop_idx)
				###traverse backward
				for i in range(trak_start_idx,len(curve_js)):

					qddot=(qdot_act[i-1]-qdot[i])/dt[i]

					if np.any(np.abs(qddot)>qddot_lim):
						alpha=fminbound(q_linesearch,0,1,args=(qdot_act[i-1],qdot[i],dt[i],qddot_lim))
					else:
						if i>large_drop_idx:
							reverse=True
							break
						alpha=1
					# print(i,alpha)
					qdot_act[i]=alpha*qdot[i]
					alpha_all[i]=alpha

					####update speed
					pose1_now=robot1.fwd(curve_js1[i])
					pose2_now=robot2.fwd(curve_js2[i])
					pose2_world_now=robot2.fwd(curve_js2[i],base2_R,base2_p)

					J1=robot1.jacobian(curve_js1[i])
					J2=robot2.jacobian(curve_js2[i])

					J1p=np.dot(pose2_world_now.R.T,J1[3:,:])
					J2p=np.dot(pose2_now.R.T,J2[3:,:])
					J2R=np.dot(pose2_now.R.T,J2[:3,:])

					p12_2=pose2_world_now.R.T@(pose1_now.p-pose2_world_now.p)
					J_p=np.hstack((J1p,-J2p+hat(p12_2)@J2R))
					speed[i]=np.linalg.norm(J_p@qdot_act[i])


					speed1[i]=np.linalg.norm(J1p@qdot_act[i][:6])
					speed2[i]=np.linalg.norm(J2p@qdot_act[i][6:])

		# plt.plot(speed)
		# plt.show()

		iteration_num+=1
		if iteration_num>10:
			# print('exceed speed iteration')
			qdot_act=0.9*qdot_act
			if iteration_num>20:
				break
		# print(np.max(np.abs(np.diff(alpha_all))))


	###smooth out trajectory
	speed=replace_outliers2(speed,threshold=0.00001)
	speed=moving_average(speed,n=5,padding=True)

	speed1=replace_outliers2(speed1,threshold=0.00001)
	speed1=moving_average(speed1,n=5,padding=True)

	speed2=replace_outliers2(speed2,threshold=0.00001)
	speed2=moving_average(speed2,n=5,padding=True)

	#####################################################################


	return speed, speed1, speed2


def main():
	###lamdot calculation
	robot=abb6640(d=50)
	curve_js = read_csv("../data/wood/Curve_js.csv",header=None).values

	# train_data = read_csv("../constraint_solver/single_arm/trajectory/curve_pose_opt/arm1.csv", names=col_names)
	# train_data = read_csv("qsol.csv", names=col_names)
	# train_data = read_csv("../constraint_solver/single_arm/trajectory/all_theta_opt/all_theta_opt_js.csv", names=col_names)

	lam=calc_lam_js(curve_js,robot)
	
	step=10
	lam_dot=calc_lamdot(curve_js,lam,robot,step)
	plt.plot(lam[::step],lam_dot)
	plt.xlabel('path length (mm)')
	plt.ylabel('max lambda_dot')
	plt.ylim([0,4000])
	plt.title('lambda_dot vs lambda')
	plt.show()

def main2():
	###lamdot dual calculation
	###read in points
	curve_js1 = read_csv("../constraint_solver/dual_arm/trajectory/arm1.csv",header=None).values
	curve_js2 = read_csv("../constraint_solver/dual_arm/trajectory/arm2.csv",header=None).values
	###define robots
	robot1=abb1200(d=50)
	robot2=abb6640()

	###read in robot2 pose
	with open('../constraint_solver/dual_arm/trajectory/abb6640.yaml') as file:
		H_6640 = np.array(yaml.safe_load(file)['H'],dtype=np.float64)

	base2_R=H_6640[:-1,:-1]
	base2_p=1000.*H_6640[:-1,-1]
	
	lam=calc_lam_js_2arm(curve_js1,curve_js2,robot1,robot2,base2_R,base2_p)
	step=10
	lam_dot=calc_lamdot_2arm(np.hstack((curve_js1,curve_js2)),lam,robot1,robot2,step)
	plt.plot(lam[::step],lam_dot)
	plt.xlabel('path length (mm)')
	plt.ylabel('max lambda_dot')
	plt.title('lambda_dot vs lambda')
	plt.ylim([0,2000])
	plt.show()

def main3():
	###speed estimation
	from MotionSend import MotionSend
	from blending import form_traj_from_bp,blend_js_from_primitive
	dataset='wood/'
	solution_dir='baseline/'
	robot=abb6640(d=50, acc_dict_path='robot_info/6640acc_new.pickle')
	curve = read_csv('../data/'+dataset+solution_dir+'/Curve_in_base_frame.csv',header=None).values

	curve_js=read_csv('../data/'+dataset+solution_dir+'/Curve_js.csv',header=None).values
	curve=curve[::1000]
	curve_js=curve_js[::1000]
	lam_original=calc_lam_cs(curve)

	ms = MotionSend()
	
	breakpoints,primitives,p_bp,q_bp=ms.extract_data_from_cmd('../data/'+dataset+'baseline/100L/command.csv')
	# breakpoints,primitives,p_bp,q_bp=ms.extract_data_from_cmd('../ILC/max_gradient/curve1_250_100L_multipeak/command.csv')
	# breakpoints,primitives,p_bp,q_bp=ms.extract_data_from_cmd('../ILC/max_gradient/curve2_1100_100L_multipeak/command.csv')

	curve_interp, curve_R_interp, curve_js_interp, breakpoints_blended=form_traj_from_bp(q_bp,primitives,robot)
	curve_js_blended,curve_blended,curve_R_blended=blend_js_from_primitive(curve_interp, curve_js_interp, breakpoints_blended, primitives,robot,zone=10)
	lam_blended=calc_lam_js(curve_js_blended,robot)

	exe_dir='../simulation/robotstudio_sim/scripts/fitting_output/'+dataset+'/100L/'
	v_cmds=[200,250,300,350,400,450,500]
	# v_cmds=[1300]
	# v_cmds=[800,900,1000,1100,1200,1300,1400]
	for v_cmd in v_cmds:

		###read actual exe file
		df = read_csv(exe_dir+"curve_exe"+"_v"+str(v_cmd)+"_z10.csv")
		lam_exe, curve_exe, curve_exe_R,curve_exe_js, act_speed, timestamp=ms.logged_data_analysis(robot,df,realrobot=True)
		lam_exe, curve_exe, curve_exe_R,curve_exe_js, act_speed, timestamp=ms.chop_extension(curve_exe, curve_exe_R,curve_exe_js, act_speed, timestamp,curve[0,:3],curve[-1,:3])

		###get joint acceleration at each pose
		joint_acc_limit=robot.get_acc(curve_exe_js)

		speed_est=traj_speed_est(robot,curve_exe_js,lam_exe,v_cmd)


		qdot_all=np.gradient(curve_exe_js,axis=0)/np.tile([np.gradient(timestamp)],(6,1)).T
		qddot_all=np.gradient(qdot_all,axis=0)/np.tile([np.gradient(timestamp)],(6,1)).T
		qddot_violate_idx=np.array([])
		for i in range(len(curve_exe_js[0])):
			qddot_violate_idx=np.append(qddot_violate_idx,np.argwhere(np.abs(qddot_all[:,i])>joint_acc_limit[:,i]))
		qddot_violate_idx=np.unique(qddot_violate_idx).astype(int)
		# for idx in qddot_violate_idx:
		# 	plt.axvline(x=lam_exe[idx],c='orange')
		

		plt.plot(lam_exe,speed_est,label='estimated')
		plt.plot(lam_exe,act_speed,label='actual')

		plt.legend()
		plt.ylim([0,1.2*v_cmd])
		plt.xlabel('lambda (mm)')
		plt.ylabel('Speed (mm/s)')
		plt.title('Speed Estimation for v'+str(v_cmd))
		plt.show()

		plt.figure()
		ax = plt.axes(projection='3d')
		ax.plot3D(curve[:,0], curve[:,1], curve[:,2], c='gray',label='original')
		ax.plot3D(curve_exe[:,0], curve_exe[:,1], curve_exe[:,2], c='red',label='execution')
		ax.scatter3D(curve_exe[qddot_violate_idx,0], curve_exe[qddot_violate_idx,1], curve_exe[qddot_violate_idx,2], c=curve_exe[qddot_violate_idx,2], cmap='Greens',label='commanded points')
		plt.show()


		


if __name__ == "__main__":
	main3()
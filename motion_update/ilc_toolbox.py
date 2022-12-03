from copy import deepcopy
import numpy as np
from general_robotics_toolbox import *
from pandas import read_csv
import sys

sys.path.append('../toolbox')
from robots_def import *
from error_check import *
# from MotionSend import *
from lambda_calc import *
from blending import *

class ilc_toolbox(object):
	def __init__(self,robot,primitives):
		#robot: single robot or tuple
		#primitives: series of primitives or list of 2 
		self.robot=robot
		self.primitives=primitives
	def interp_trajectory(self,q_bp,zone):
		curve_interp, curve_R_interp, curve_js_interp, breakpoints_blended=form_traj_from_bp(q_bp,self.primitives,self.robot)
		return blend_js_from_primitive(curve_interp, curve_js_interp, breakpoints_blended, self.primitives,self.robot,zone=zone)

	def get_q_bp(self,breakpoints,curve_fit_js):
		q_bp=[]
		for i in range(len(self.primitives)):
			if self.primitives[i]=='movej_fit':
				q_bp.append(points_list[i])
			elif self.primitives[i]=='movel_fit':
				q_bp.append(car2js(self.robot,curve_fit_js[breakpoints[i]],np.array(points_list[i]),self.robot.fwd(curve_fit_js[breakpoints[i]]).R)[0])
			else:
				q_bp.append([car2js(self.robot,curve_fit_js[int((breakpoints[i]+breakpoints[i-1])/2)],points_list[i][0],self.robot.fwd(curve_fit_js[int((breakpoints[i]+breakpoints[i-1])/2)]).R)[0]\
					,car2js(self.robot,curve_fit_js[breakpoints[i]],points_list[i][0],self.robot.fwd(curve_fit_js[breakpoints[i]]).R)[0]])
		return q_bp

	
	def get_gradient_from_model_xyz(self,p_bp,q_bp,breakpoints_blended,curve_blended,max_error_curve_blended_idx,worst_point_pose,closest_p,breakpoint_interp_2tweak_indices):
		
		###p_bp:								xyz configs at breakpoints
		###q_bp:								joint configs at breakpoints
		###breakpoints_blended:					breakpoints of blended trajectory
		###curve_blended:						blended trajectory
		###max_error_curve_blended_idx:	p', 	closest point to worst case error on blended trajectory
		###worst_point_pose:					execution curve with worst case pose
		###closest_p:							closest point on original curve
		###breakpoint_interp_2tweak_indices:	closest N breakpoints

		de_dp=[]    #de_dp1q1,de_dp1q2,...,de_dp3q6
		delta=0.2 	#mm

		###len(primitives)==len(breakpoints)==len(breakpoints_blended)==len(points_list)
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			for bp_sub_idx in range(len(p_bp[m])):
				# print(np.linalg.norm(p_bp[m][bp_sub_idx]-closest_p))
				for n in range(3): #3DOF, xyz
					q_bp_temp=np.array(copy.deepcopy(q_bp))
					p_bp_temp=copy.deepcopy(p_bp)
					p_bp_temp[m][bp_sub_idx][n]+=delta

					q_bp_temp[m][bp_sub_idx]=car2js(self.robot,q_bp[m][bp_sub_idx],np.array(p_bp_temp[m][bp_sub_idx]),self.robot.fwd(q_bp[m][bp_sub_idx]).R)[0]

					#restore new trajectory, only for adjusted breakpoint, 1-bp change requires traj interp from 5 bp
					short_version=range(max(m-2,0),min(m+3,len(breakpoints_blended)))
					###start & end idx, choose points in the middle of breakpoints to avoid affecting previous/next blending segments, unless at the boundary (star/end of all curve)
					###guard 5 breakpoints for short blending
					if short_version[0]==0:
						short_version=range(0,5)
						start_idx=breakpoints_blended[short_version[0]]
					else:
						start_idx=int((breakpoints_blended[short_version[0]]+breakpoints_blended[short_version[1]])/2)
					if short_version[-1]==len(breakpoints_blended)-1:
						short_version=range(len(breakpoints_blended)-5,len(breakpoints_blended))
						end_idx = breakpoints_blended[short_version[-1]]+1
					else:
						end_idx=int((breakpoints_blended[short_version[-1]]+breakpoints_blended[short_version[-2]])/2)+1


					curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp=form_traj_from_bp(q_bp_temp[short_version],[self.primitives[i] for i in short_version],self.robot)

					curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[i] for i in short_version],self.robot,zone=10)
					
					curve_blended_new=copy.deepcopy(curve_blended)

					

					curve_blended_new[start_idx:end_idx]=curve_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]

					###calculate relative gradient
					worst_case_point_shift=curve_blended_new[max_error_curve_blended_idx]-curve_blended[max_error_curve_blended_idx]

					###get new error - prev error
					de=np.linalg.norm(worst_point_pose.p+worst_case_point_shift-closest_p)-np.linalg.norm(worst_point_pose.p-closest_p)

					de_dp.append(de/delta)

		de_dp=np.reshape(de_dp,(-1,1))
		# print(de_dp)

		return de_dp

	def get_gradient_from_model_ori(self,p_bp,q_bp,breakpoints_blended,curve_blended_R,max_error_curve_blended_idx,worst_point_pose,closest_N,breakpoint_interp_2tweak_indices):
		
		###p_bp:								xyz configs at breakpoints
		###q_bp:								joint configs at breakpoints
		###breakpoints_blended:					breakpoints of blended trajectory
		###curve_blended_R:						blended trajectory of R
		###max_error_curve_blended_idx:	p', 	closest point to worst case error on blended trajectory
		###worst_point_pose:					execution curve with worst case pose
		###closest_N:							Normal of closest point on original curve
		###breakpoint_interp_2tweak_indices:	closest N breakpoints

		de_ori_dp=[]
		delta=0.01 	#rad

		###len(primitives)==len(breakpoints)==len(breakpoints_blended)==len(points_list)
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			for n in range(3): #3DOF, xyz
				q_bp_temp=np.array(copy.deepcopy(q_bp))
				k_temp=np.zeros(3)
				k_temp[n]=1
				R_new=rot(k_temp,delta)@self.robot.fwd(q_bp[m][-1]).R

				q_bp_temp[m][-1]=car2js(self.robot,q_bp[m][-1],p_bp[m][-1],R_new)[0]

				#restore new trajectory, only for adjusted breakpoint, 1-bp change requires traj interp from 5 bp
				short_version=range(max(m-2,0),min(m+3,len(breakpoints_blended)))
				###start & end idx, choose points in the middle of breakpoints to avoid affecting previous/next blending segments, unless at the boundary (star/end of all curve)
				###guard 5 breakpoints for short blending
				if short_version[0]==0:
					short_version=range(0,5)
					start_idx=breakpoints_blended[short_version[0]]
				else:
					start_idx=int((breakpoints_blended[short_version[0]]+breakpoints_blended[short_version[1]])/2)
				if short_version[-1]==len(breakpoints_blended)-1:
					short_version=range(len(breakpoints_blended)-5,len(breakpoints_blended))
					end_idx = breakpoints_blended[short_version[-1]]+1
				else:
					end_idx=int((breakpoints_blended[short_version[-1]]+breakpoints_blended[short_version[-2]])/2)+1


				curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp=form_traj_from_bp(q_bp_temp[short_version],[self.primitives[i] for i in short_version],self.robot)

				curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[i] for i in short_version],self.robot,zone=10)
				
				curve_blended_new_R=copy.deepcopy(curve_blended_R)

				

				curve_blended_new_R[start_idx:end_idx]=curve_R_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_R_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]

				###calculate dR
				worst_case_point_shift_R=curve_blended_new_R[max_error_curve_blended_idx]@curve_blended_R[max_error_curve_blended_idx].T

				###get new error - prev error
				de_ori=get_angle(worst_case_point_shift_R@worst_point_pose.R[:,-1],closest_N)-get_angle(worst_point_pose.R[:,-1],closest_N)

				de_ori_dp.append(de_ori/delta)

		de_ori_dp=np.reshape(de_ori_dp,(-1,1))

		return de_ori_dp
	
	def get_gradient_from_model_xyz_fanuc(self,p_bp,q_bp,breakpoints_blended,curve_blended,max_error_curve_blended_idx,worst_point_pose,closest_p,breakpoint_interp_2tweak_indices,speed):
		
		###p_bp:								xyz configs at breakpoints
		###q_bp:								joint configs at breakpoints
		###breakpoints_blended:					breakpoints of blended trajectory
		###curve_blended:						blended trajectory
		###max_error_curve_blended_idx:	p', 	closest point to worst case error on blended trajectory
		###worst_point_pose:					execution curve with worst case pose
		###closest_p:							closest point on original curve
		###breakpoint_interp_2tweak_indices:	closest N breakpoints

		###TODO:ADD MOVEC SUPPORT
		de_dp=[]    #de_dp1q1,de_dp1q2,...,de_dp3q6
		de_ori_dp=[]
		delta=0.1 	#mm

		if type(self.robot) is list:
			robot=self.robot[0]
			primitives=self.primitives[0]
		else:
			robot=self.robot
			primitives=self.primitives
			

		###len(primitives)==len(breakpoints)==len(breakpoints_blended)==len(points_list)
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			for n in range(3): #3DOF, xyz
				q_bp_temp=np.array(copy.deepcopy(q_bp))
				p_bp_temp=copy.deepcopy(p_bp)
				p_bp_temp[m][0][n]+=delta

				q_bp_temp[m][0]=car2js(robot,q_bp[m][0],np.array(p_bp_temp[m][0]),robot.fwd(q_bp[m][0]).R)[0]###TODO:ADD MOVEC SUPPORT

				#restore new trajectory, only for adjusted breakpoint, 1-bp change requires traj interp from 5 bp
				short_version=range(max(m-2,0),min(m+3,len(breakpoints_blended)))
				###start & end idx, choose points in the middle of breakpoints to avoid affecting previous/next blending segments, unless at the boundary (star/end of all curve)
				###guard 5 breakpoints for short blending
				if short_version[0]==0:
					short_version=range(0,5)
					start_idx=breakpoints_blended[short_version[0]]
				else:
					start_idx=int((breakpoints_blended[short_version[0]]+breakpoints_blended[short_version[1]])/2)
				if short_version[-1]==len(breakpoints_blended)-1:
					short_version=range(len(breakpoints_blended)-5,len(breakpoints_blended))
					end_idx = breakpoints_blended[short_version[-1]]+1
				else:
					end_idx=int((breakpoints_blended[short_version[-1]]+breakpoints_blended[short_version[-2]])/2)+1


				curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp=form_traj_from_bp(q_bp_temp[short_version],[primitives[i] for i in short_version],robot)

				# curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_cart_from_primitive(curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [primitives[i] for i in short_version],self.robot,speed)
				curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [primitives[i] for i in short_version],robot,zone=10)
				
				curve_blended_new=copy.deepcopy(curve_blended)

				

				curve_blended_new[start_idx:end_idx]=curve_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]

				###calculate relative gradient
				worst_case_point_shift=curve_blended_new[max_error_curve_blended_idx]-curve_blended[max_error_curve_blended_idx]

				###get new error - prev error
				de=np.linalg.norm(worst_point_pose.p+worst_case_point_shift-closest_p)-np.linalg.norm(worst_point_pose.p-closest_p)

				de_dp.append(de/delta)

		de_dp=np.reshape(de_dp,(-1,1))

		return de_dp


	# def get_speed_gradient_from_model(self,q_bp,breakpoints_blended,curve_js_blended,curve_blended,min_speed_curve_blended_idx,breakpoint_interp_2tweak_indices,speed_est,vd):
		
	# 	###q_bp:								joint configs at breakpoints
	# 	###breakpoints_blended:					breakpoints of blended trajectory
	# 	###curve_js_blended:					blended trajectory js
	# 	###curve_blended:						blended trajectory
	# 	###min_speed_curve_blended_idx:	p', 	closest point to worst case error on blended trajectory
	# 	###breakpoint_interp_2tweak_indices:	closest N breakpoints
	# 	###speed_est:							estimated speed for blended trajectory
	# 	###vd:									desired TCP velocity

	# 	###TODO:ADD MOVEC SUPPORT
	# 	dv_dq=[]    #ds_dp1q1,dv_dp1q2,...,dev_dp3q6
	# 	delta=0.01 	#rad

	# 	###len(primitives)==len(breakpoints)==len(breakpoints_blended)==len(points_list)
	# 	for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
	# 		for n in range(len(q_bp[0][0])): #6DOF joints
	# 			q_bp_temp=np.array(copy.deepcopy(q_bp))
	# 			q_bp_temp[m][0][n]+=delta

	# 			#restore new trajectory, only for adjusted breakpoint, 1-bp change requires traj interp from 5 bp
	# 			short_version=range(max(m-2,0),min(m+3,len(breakpoints_blended)))
	# 			###start & end idx, choose points in the middle of breakpoints to avoid affecting previous/next blending segments, unless at the boundary (star/end of all curve)
	# 			###guard 5 breakpoints for short blending
	# 			if short_version[0]==0:
	# 				short_version=range(0,5)
	# 				start_idx=breakpoints_blended[short_version[0]]
	# 			else:
	# 				start_idx=int((breakpoints_blended[short_version[0]]+breakpoints_blended[short_version[1]])/2)
	# 			if short_version[-1]==len(breakpoints_blended)-1:
	# 				short_version=range(len(breakpoints_blended)-5,len(breakpoints_blended))
	# 				end_idx = breakpoints_blended[short_version[-1]]+1
	# 			else:
	# 				end_idx=int((breakpoints_blended[short_version[-1]]+breakpoints_blended[short_version[-2]])/2)+1

	# 			###form new blended trajectory
	# 			curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp=form_traj_from_bp(q_bp_temp[short_version],[self.primitives[i] for i in short_version],self.robot)
	# 			curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[i] for i in short_version],self.robot,zone=10)
	# 			curve_blended_new=copy.deepcopy(curve_blended)
	# 			curve_blended_new[start_idx:end_idx]=curve_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]
	# 			curve_js_blended_new=copy.deepcopy(curve_js_blended)
	# 			curve_js_blended_new[start_idx:end_idx]=curve_js_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_js_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]

	# 			###get new speed
	# 			lam_blended_new=calc_lam_cs(curve_blended_new)
	# 			speed_est_new=traj_speed_est(self.robot,curve_js_blended_new,lam_blended_new,vd)


	# 			###get new est speed - prev est speed
	# 			dv=np.average(speed_est_new[min_speed_curve_blended_idx-5:min_speed_curve_blended_idx+5])-np.average(speed_est[min_speed_curve_blended_idx-5:min_speed_curve_blended_idx+5])

	# 			dv_dq.append(dv/delta)

	# 	dv_dq=np.reshape(dv_dq,(-1,1))
	# 	print(dv_dq)

	# 	return dv_dq


	# def get_speed_gradient_from_traj_exe(self,p_bp,q_bp,curve_exe,curve_exe_js,timestamp,lam_exe,valley,vd):
		
	# 	###p_bp:								xyz of breakpoints
	# 	###q_bp:								joint configs at breakpoints
	# 	###curve_exe:							exe_trajectory
	# 	###curve_exe_js:						exe trajectory in js
	# 	###timestamp:							exe timestamp
	# 	###lam_exe:								path length of curve_exe
	# 	###valley:								index of curve_exe at speed valley
	# 	###vd:									desired TCP velocity

	# 	dq_exe=np.gradient(curve_exe_js,axis=0)
	# 	qdot_exe=np.divide(dq_exe,np.tile(np.array([np.gradient(timestamp)]).T,(1,6)))

	# 	###find closest breakpoint to the valley
	# 	p_bp_np=np.squeeze(np.array(p_bp))
	# 	_,closest_bp=calc_error(curve_exe[valley],p_bp_np)  # index of breakpoints closest to speed valley point

	# 	###find closest points on curve_exe of neightbor breakpoints
	# 	_,valley_prev=calc_error(p_bp[closest_bp-1],curve_exe)
	# 	_,valley_next=calc_error(p_bp[closest_bp+1],curve_exe)


	# 	###form trajectory model from curve_exe_js 
	# 	curve_js_model=copy.deepcopy(curve_exe_js[valley_prev:valley_next])

	# 	for j in range(len(curve_exe_js[0])):	
	# 		poly = BPoly.from_derivatives([lam_exe[valley_prev],lam_exe[valley],lam_exe[valley_next]], \
	# 			[[curve_exe_js[valley_prev,j],(curve_exe_js[valley_prev,j]-curve_exe_js[valley_prev-1,j])/(lam_exe[valley_prev]-lam_exe[valley_prev-1])], \
	# 			[curve_exe_js[valley,j]],\
	# 			[curve_exe_js[valley_next,j],(curve_exe_js[valley_next,j]-curve_exe_js[valley_next-1,j])/(lam_exe[valley_next]-lam_exe[valley_next-1])]])

	# 		curve_js_model[:,j]=poly(lam_exe[valley_prev:valley_next])

	# 	# speed_est=traj_speed_est(self.robot,curve_js_model,lam_exe[valley_prev:valley_next],vd,qdot_init=qdot_exe[valley_prev])
	# 	speed_est=traj_speed_est(self.robot,curve_exe_js[valley_prev:valley_next],lam_exe[valley_prev:valley_next],vd,qdot_init=qdot_exe[valley_prev])

	# 	speed_act=np.linalg.norm(np.gradient(curve_exe,axis=0),axis=1)/np.gradient(timestamp)
	# 	plt.plot(lam_exe[valley_prev:valley_next],speed_act[valley_prev:valley_next],label='original')
	# 	plt.plot(lam_exe[valley_prev:valley_next],speed_est,label='estimated')
	# 	plt.legend()
	# 	plt.show()

	# 	###TODO:ADD MOVEC SUPPORT
	# 	dv_dq=[]    #dv_dp1q1,dv_dp1q2,...,dv_dp1q6
	# 	delta=0.01 	#rad


	# 	for n in range(len(q_bp[0][0])): #6DOF joints
	# 		q_bp_temp=np.array(copy.deepcopy(q_bp))
	# 		q_bp_temp[m][0][n]+=delta

	# 		#restore new trajectory, only for adjusted breakpoint, 1-bp change requires traj interp from 5 bp
	# 		short_version=range(max(m-2,0),min(m+3,len(breakpoints_blended)))
	# 		###start & end idx, choose points in the middle of breakpoints to avoid affecting previous/next blending segments, unless at the boundary (star/end of all curve)
	# 		###guard 5 breakpoints for short blending
	# 		if short_version[0]==0:
	# 			short_version=range(0,5)
	# 			start_idx=breakpoints_blended[short_version[0]]
	# 		else:
	# 			start_idx=int((breakpoints_blended[short_version[0]]+breakpoints_blended[short_version[1]])/2)
	# 		if short_version[-1]==len(breakpoints_blended)-1:
	# 			short_version=range(len(breakpoints_blended)-5,len(breakpoints_blended))
	# 			end_idx = breakpoints_blended[short_version[-1]]+1
	# 		else:
	# 			end_idx=int((breakpoints_blended[short_version[-1]]+breakpoints_blended[short_version[-2]])/2)+1

	# 		###form new blended trajectory
	# 		curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp=form_traj_from_bp(q_bp_temp[short_version],[self.primitives[i] for i in short_version],self.robot)
	# 		curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[i] for i in short_version],self.robot,zone=10)
	# 		curve_blended_new=copy.deepcopy(curve_blended)
	# 		curve_blended_new[start_idx:end_idx]=curve_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]
	# 		curve_js_blended_new=copy.deepcopy(curve_js_blended)
	# 		curve_js_blended_new[start_idx:end_idx]=curve_js_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_js_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]

	# 		###get new speed
	# 		lam_blended_new=calc_lam_cs(curve_blended_new)
	# 		speed_est_new=traj_speed_est2(self.robot,curve_js_blended_new,lam_blended_new,vd)


	# 		###get new est speed - prev est speed
	# 		dv=np.average(speed_est_new[min_speed_curve_blended_idx-5:min_speed_curve_blended_idx+5])-np.average(speed_est[min_speed_curve_blended_idx-5:min_speed_curve_blended_idx+5])

	# 		dv_dq.append(dv/delta)

	# 	dv_dq=np.reshape(dv_dq,(-1,1))
	# 	print(dv_dq)

	# 	return dv_dq



	# def update_bp_speed(self,p_bp,q_bp,dv_dq,min_speed,breakpoint_interp_2tweak_indices,vd,alpha=0.5):
	# 	###p_bp:								xyz of breakpoints
	# 	###q_bp:								joint configs of breakpoints
	# 	###dv_dq								gradient of speed and each breakpoints
	# 	###min_speed:							lowest speed value
	# 	###breakpoint_interp_2tweak_indices:	closest N breakpoints
	# 	###alpha:								stepsize

	# 	q_adjustment=alpha*np.linalg.pinv(dv_dq)*(min_speed-vd)

	# 	for i in range(len(breakpoint_interp_2tweak_indices)):  #3 breakpoints
	# 		q_bp[breakpoint_interp_2tweak_indices[i]][0]+=q_adjustment[0][6*i:6*(i+1)]
	# 		###TODO:ADD MOVEC SUPPORT
	# 		p_bp[breakpoint_interp_2tweak_indices[i]][0]=self.robot.fwd(q_bp[breakpoint_interp_2tweak_indices[i]][0]).p

	# 	return p_bp, q_bp

	def get_gradient_from_model_xyz_dual(self,\
											p_bp,q_bp,breakpoints_blended,curve_blended,max_error_curve_blended_idx,worst_point_joints,closest_p,breakpoint_interp_2tweak_indices):
		
		###p_bp:								[p_bp1,p_bp2],xyz configs at breakpoints
		###q_bp:								[q_bp1,q_bp2],joint configs at breakpoints
		###breakpoints_blended:					breakpoints of blended trajectory
		###curve_blended:						[curve_blended1,curve_blended2],blended trajectory
		###max_error_curve_blended_idx:	p', 	closest point to worst case error on blended trajectory
		###worst_point_joints:					[worst_point_joint1,worst_point_joint2] execution curve with worst case pose
		###closest_p:							closest point on relative path
		###breakpoint_interp_2tweak_indices:	closest N breakpoints

		robot1_worst_pose=self.robot[0].fwd(worst_point_joints[0])
		robot2_worst_pose=self.robot[-1].fwd(worst_point_joints[-1])
		robot2_worst_pose_global=self.robot[-1].fwd(worst_point_joints[-1],world=True)
		worst_point_relative_p=robot2_worst_pose_global.R.T@(robot1_worst_pose.p-robot2_worst_pose_global.p)

		worst_case_error=np.linalg.norm(worst_point_relative_p-closest_p)

		delta=0.1 	##mm
		###TODO:ADD MOVEC SUPPORT
		de_dp=[]    #(de_dp1q1,de_dp1q2,...,de_dp3q6)1,(de_dp1q1,de_dp1q2,...,de_dp3q6)2
		de_ori_dp=[]


		###len(primitives)==len(breakpoints)==len(breakpoints_blended)==len(points_list)
		for r in range(2):		#2 robots	
			for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
				for n in range(3): #3DOF, xyz
					q_bp_temp=np.array(copy.deepcopy(q_bp[r]))
					p_bp_temp=copy.deepcopy(p_bp[r])

					p_bp_temp[m][0][n]+=delta


					q_bp_temp[m][0]=car2js(self.robot[r],q_bp[r][m][0],np.array(p_bp_temp[m][0]),self.robot[r].fwd(q_bp[r][m][0]).R)[0]###TODO:ADD MOVEC SUPPORT

					#restore new trajectory, only for adjusted breakpoint, 1-bp change requires traj interp from 5 bp
					short_version=range(max(m-3,0),min(m+4,len(breakpoints_blended)))
					###start & end idx, choose points in the middle of breakpoints to avoid affecting previous/next blending segments, unless at the boundary (star/end of all curve)
					###guard 7 breakpoints for short blending, minimum is 7 instead of 5 if adjusting 1 bp and check changes of surrunding 3 bp region due to blending

					if short_version[0]==0:
						short_version=range(0,7)
						start_idx=breakpoints_blended[short_version[0]]
					else:
						start_idx=int((breakpoints_blended[short_version[1]]+breakpoints_blended[short_version[2]])/2)
					if short_version[-1]==len(breakpoints_blended)-1:
						short_version=range(len(breakpoints_blended)-7,len(breakpoints_blended))
						end_idx = breakpoints_blended[short_version[-1]]+1
					else:
						end_idx=int((breakpoints_blended[short_version[-2]]+breakpoints_blended[short_version[-3]])/2)+1


					curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp=form_traj_from_bp(q_bp_temp[short_version],[self.primitives[r][i] for i in short_version],self.robot[r])

					curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[r][i] for i in short_version],self.robot[r],zone=10)
					
					curve_blended_new=copy.deepcopy(curve_blended[r])

					curve_blended_new[start_idx:end_idx]=curve_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]

					###calculate relative gradient
					worst_case_point_shift=curve_blended_new[max_error_curve_blended_idx]-curve_blended[r][max_error_curve_blended_idx]
					


					###convert shifted delta in robot2 tool frame
					if r==0:	#for robot 1, curve is in global frame
						worst_case_point_shift=robot2_worst_pose_global.R.T@worst_case_point_shift
					else:		#for robot2, curve is in robot2 base frame, relative motion, direction reversed
						worst_case_point_shift=robot2_worst_pose.R.T@(-worst_case_point_shift)

					

					###get new error - prev error
					de=np.linalg.norm(worst_point_relative_p+worst_case_point_shift-closest_p)-worst_case_error
					de_dp.append(de/delta)

		de_dp=np.reshape(de_dp,(-1,1))

		return de_dp

	def update_bp_xyz(self,p_bp,q_bp,de_dp,max_error,breakpoint_interp_2tweak_indices,alpha=0.5):
		###p_bp:								xyz of breakpoints
		###q_bp:								joint configs of breakpoints
		###de_dp；								gradient of error and each breakpoints
		###max_error:							worst case error value
		###breakpoint_interp_2tweak_indices:	closest N breakpoints
		###alpha:								stepsize

		p_bp_temp = deepcopy(p_bp)
		q_bp_temp = deepcopy(q_bp)

		if type(self.robot) is list:
			robot=self.robot[0]
		else:
			robot=self.robot

		point_adjustment=-alpha*np.linalg.pinv(de_dp)*max_error
		# print(max_error,point_adjustment)
		idx=0
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			for bp_sub_idx in range(len(p_bp[m])):

				p_bp_temp[m][bp_sub_idx]+=point_adjustment[0][3*idx:3*(idx+1)]
				q_bp_temp[m][bp_sub_idx]=car2js(robot,q_bp[m][bp_sub_idx],p_bp_temp[m][bp_sub_idx],robot.fwd(q_bp[m][bp_sub_idx]).R)[0]
				idx+=1
		return p_bp_temp, q_bp_temp


	def update_bp_ori(self,p_bp,q_bp,de_ori_dp,max_angle,breakpoint_interp_2tweak_indices,alpha=0.5):
		###p_bp:								xyz of breakpoints
		###q_bp:								joint configs of breakpoints
		###de_ori_dp:							gradient of ori error and each breakpoints
		###max_angle:							worst case error value
		###breakpoint_interp_2tweak_indices:	closest N breakpoints
		###alpha:								stepsize

		ori_adjustment=-alpha*np.linalg.pinv(de_ori_dp)*max_angle
		# print(max_error,ori_adjustment)
		idx=0
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			R_old=self.robot.fwd(q_bp[m][-1]).R
			w_temp=ori_adjustment[0][3*idx:3*(idx+1)]
			theta_temp=np.linalg.norm(w_temp)
			if theta_temp==0:
				continue

			k_temp=w_temp/theta_temp
			R_new=rot(k_temp,alpha*theta_temp)@R_old
			q_bp[m][-1]=car2js(self.robot,q_bp[m][-1],p_bp[m][-1],R_new)[0]
			idx+=1
		return q_bp


	def update_bp_xyz_dual(self,p_bp,q_bp,de_dp,max_error,breakpoint_interp_2tweak_indices,alpha=0.5):
		###q_bp:								[q_bp1,q_bp2],joint configs at breakpoints
		###p_bp:								[p_bp1,p_bp2],xyz configs at breakpoints
		###de_dp；								gradient of error and each breakpoints
		###max_error:							worst case error value
		###breakpoint_interp_2tweak_indices:	closest N breakpoints
		###alpha:								stepsize
		p_bp_new=copy.deepcopy(p_bp)
		q_bp_new=copy.deepcopy(q_bp)
		point_adjustment=-alpha*np.linalg.pinv(de_dp)*max_error
		for r in range(2):
			for i in range(len(breakpoint_interp_2tweak_indices)):  #3 breakpoints
				p_bp_new[r][breakpoint_interp_2tweak_indices[i]][0]+=point_adjustment[0][len(breakpoint_interp_2tweak_indices)*3*r+3*i:len(breakpoint_interp_2tweak_indices)*3*r+3*(i+1)]
				###TODO:ADD MOVEC SUPPORT
				q_bp_new[r][breakpoint_interp_2tweak_indices[i]][0]=car2js(self.robot[r],q_bp[r][breakpoint_interp_2tweak_indices[i]][0],p_bp_new[r][breakpoint_interp_2tweak_indices[i]][0],self.robot[r].fwd(q_bp[r][breakpoint_interp_2tweak_indices[i]][0]).R)[0]

		return p_bp_new[0], q_bp_new[0], p_bp_new[1], q_bp_new[1]


	def get_gradient_from_model_6j(self,q_bp,breakpoints_blended,curve_blended,curve_R_blended,max_error_curve_blended_idx,worst_point_pose,closest_p,closest_N,breakpoint_interp_2tweak_indices):
		###q_bp:								joint configs at breakpoints
		###breakpoints_blended:					breakpoints of blended trajectory
		###curve_blended:						blended trajectory
		###curve_R_blended:						blended trajectory
		###max_error_curve_blended_idx:	p', 	closest point to worst case error on blended trajectory
		###worst_point_pose:					execution curve with worst case pose
		###closest_p:							closest point on original curve
		###closest_N:							closest point on original curve, its normal
		###breakpoint_interp_2tweak_indices:	closest N breakpoints

		de_dp=[]    #de_dp1q1,de_dp1q2,...,de_dp3q6
		de_ori_dp=[]
		delta=0.01 	#rad


		###len(primitives)==len(breakpoints)==len(breakpoints_blended)==len(points_list)
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			for n in range(6): #6DOF, q1~q6
				q_bp_temp=np.array(copy.deepcopy(q_bp))
				q_bp_temp[m][0][n]+=delta		###ADD MOVEC SUPPORT
				
				#restore new trajectory, only for adjusted breakpoint, 1-bp change requires traj interp from 5 bp
				short_version=range(max(m-2,0),min(m+2,len(breakpoints_blended)-1))						
				curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp=form_traj_from_bp(q_bp_temp[short_version],[self.primitives[i] for i in short_version],self.robot)

				curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[i] for i in short_version],self.robot,zone=10)
				
				curve_blended_new=copy.deepcopy(curve_blended)
				curve_R_blended_new=copy.deepcopy(curve_R_blended)
				start_idx=int((breakpoints_blended[short_version[0]]+breakpoints_blended[short_version[1]])/2)
				end_idx=int((breakpoints_blended[short_version[-1]]+breakpoints_blended[short_version[-2]])/2)

				curve_blended_new[start_idx:end_idx]=curve_blended_temp[start_idx-breakpoints_blended[short_version[0]]:-1-(breakpoints_blended[short_version[-1]]-end_idx)]
				curve_R_blended_new[start_idx:end_idx]=curve_R_blended_temp[start_idx-breakpoints_blended[short_version[0]]:-1-(breakpoints_blended[short_version[-1]]-end_idx)]

				###calculate relative gradient, xyz
				worst_case_point_shift=curve_blended_new[max_error_curve_blended_idx]-curve_blended[max_error_curve_blended_idx]
				###get new error
				de=np.linalg.norm(worst_point_pose.p+worst_case_point_shift-closest_p)-np.linalg.norm(worst_point_pose.p-closest_p)

				###calculate relative gradient, ori
				worst_case_R_shift=curve_R_blended_new[max_error_curve_blended_idx]@curve_R_blended[max_error_curve_blended_idx].T

				de_ori=get_angle(worst_case_R_shift[:,-1]@worst_point_pose.R,closest_N)-get_angle(worst_point_pose.R[:,-1],closest_N)

				de_dp.append(de/delta)
				de_ori_dp.append(de_ori/delta)

		de_dp=np.reshape(de_dp,(-1,1))
		de_ori_dp=np.reshape(de_ori_dp,(-1,1))

		return de_dp, de_ori_dp

	def update_bp_6j(self,p_bp,q_bp,de_dp,de_ori_dp,max_error,max_ori_error,breakpoint_interp_2tweak_indices,alpha1=0.5,alpha2=0.1):
		###p_bp:								xyz of breakpoints
		###q_bp:								joint configs of breakpoints
		###de_dp:								gradient of error and each breakpoints
		###de_ori_dp:							gradient of orientation error and each breakpoints
		###max_error:							worst case error value
		###max_ori_error:						worst case error angle
		###breakpoint_interp_2tweak_indices:	closest N breakpoints
		###alpha1, alpha2:						stepsize for xyz and orientation

		bp_q_adjustment=-alpha1*np.linalg.pinv(de_dp)*max_error-alpha2*np.linalg.pinv(de_ori_dp)*max_ori_error
		for i in range(len(breakpoint_interp_2tweak_indices)):  #3 breakpoints
			q_bp[breakpoint_interp_2tweak_indices[i]][0]+=bp_q_adjustment[0][6*i:6*(i+1)]
			p_bp[breakpoint_interp_2tweak_indices[i]][0]=self.robot.fwd(q_bp[breakpoint_interp_2tweak_indices[i]][0]).p

		return p_bp, q_bp


	def update_bp_ilc_cart(self,p_bp,q_bp,exe_bp_p,exe_bp_R,exe_bp_p_new, exe_bp_R_new,alpha1=0.5,alpha2=0.5):
		###fixing initial and end breakpoint
		###TODO, add rotation gradient

		###p_bp:								xyz of breakpoints, u
		###q_bp:								joint configs of breakpoints, u
		###exe_bp_p:							breakpoints of execution, position, output y
		###exe_bp_R:							breakpoints of execution, R, output y
		###exe_bp_p_new:						breakpoints of execution with augmented input u', position, output y'
		###exe_bp_R_new:						breakpoints of execution with augmented input u', R, 		output y'

		for i in range(1,len(p_bp)-1):
			###get new ek
			grad_ep=exe_bp_p_new[i-1]-exe_bp_p[i-1]
			###time reverse
			reverse_idx=-1-i
			p_bp[reverse_idx]-=alpha1*grad_ep


			q_bp[reverse_idx][0]=car2js(self.robot,q_bp[reverse_idx][0],p_bp[reverse_idx][0],self.robot.fwd(q_bp[reverse_idx][0]).R)[0]
		return p_bp,q_bp

	def update_bp_ilc_js(self,q_bp,exe_bp_q,exe_bp_q_new,alpha1=0.5,alpha2=0.5):

		###q_bp:								joint configs of breakpoints, u
		###exe_bp_q:							breakpoints of execution, position, output y
		###exe_bp_q_new:						breakpoints of execution with augmented input u', position, output y'

		for i in range(1,len(p_bp)-1):
			###get new ek
			grad_eq=exe_bp_q_new[i-1]-exe_bp_q[i-1]
			###time reverse
			reverse_idx=-1-i
			q_bp[reverse_idx]-=alpha1*grad_eq


		return q_bp


	def sto_gradient_from_model(self,p_bp,q_bp,total_points=100,K=20):
		p_bp=np.array(p_bp)
		now=time.time()
		curve_interp, curve_R_interp, curve_js_interp, breakpoints_blended=form_traj_from_bp(q_bp,self.primitives,self.robot)
		curve_js_blended,curve_blended,curve_R_blended=blend_js_from_primitive(curve_interp, curve_js_interp, breakpoints_blended, self.primitives,self.robot,zone=10)
		print('time for 1 interpolation: ',time.time()-now)

		###downsample model interploated curve
		step_size=int(len(curve_blended)/total_points)
		curve_blended_downsampled=curve_blended[::step_size]
		#### runs to get stochastic gradient
		dp_model_all=[]
		d_bp_p_all=[]
		for k in range(K):
			# print(k,'th iteration')
			d_bp_p=np.random.uniform(low=-0.1,high=0.1,size=p_bp.shape)	#changes in position of breakpoints

			p_bp_new=p_bp+d_bp_p

			###find inv of new bp's
			q_bp_new=[]
			for i in range(len(p_bp)):
				q_bp_new.append(car2js(self.robot,q_bp[i][0],p_bp_new[i][0],self.robot.fwd(q_bp[i][0]).R))


			###use model to get new interpolated curve
			curve_interp_new, curve_R_interp_new, curve_js_interp_new, breakpoints_blended_new=form_traj_from_bp(q_bp_new,self.primitives,self.robot)
			_,curve_blended_new,_=blend_js_from_primitive(curve_interp_new, curve_js_interp_new, breakpoints_blended_new, self.primitives,self.robot,zone=10)

			###downsample
			curve_blended_new_downsampled=curve_blended_new[::step_size]
			dp_model=curve_blended_new_downsampled-curve_blended_downsampled

			###store delta's
			dp_model_all.append(dp_model.flatten())
			d_bp_p_all.append(d_bp_p.flatten())

		dp_model_all=np.array(dp_model_all)
		d_bp_p_all=np.array(d_bp_p_all)

		G=dp_model_all.T@np.linalg.pinv(d_bp_p_all.T)

		return curve_blended_downsampled,G

	def get_error_direction(self,curve,p_bp,q_bp,curve_exe,curve_exe_R):
		###find points on curve_exe closest to all p_bp's, and closest point on blended trajectory to all p_bp's
		bp_exe_indices=[]		###breakpoints on curve_exe
		curve_original_indices=[]
		error_bps=[]
		error_bps_v=np.zeros((len(p_bp),2,3))
		error_bps_w=np.zeros((len(p_bp),2,3))
		for bp_idx in range(len(p_bp)):
			for bp_sub_idx in range(len(p_bp[bp_idx])):
				if bp_idx==0 and bp_sub_idx==0:
					curve_original_idx=0
					error_bp,bp_exe_idx=calc_error(curve[curve_original_idx,:3],curve_exe)							###find closest point on curve_exe to bp
				elif bp_idx==len(p_bp)-1 and bp_sub_idx==len(p_bp[bp_idx])-1:
					curve_original_idx=len(curve)-1
					error_bp,bp_exe_idx=calc_error(curve[curve_original_idx,:3],curve_exe)
				else:
					_,bp_exe_idx=calc_error(p_bp[bp_idx][bp_sub_idx],curve_exe)							###find closest point on curve_exe to bp
					bp_exe_indices.append(bp_exe_idx)
					error_bp,curve_original_idx=calc_error(curve_exe[bp_exe_idx],curve[:,:3])	###find closest point on curve_original to curve_exe[bp]
					curve_original_indices.append(curve_original_idx)			
				
				error_bps.append(error_bp)
				###get error direction
				error_bps_v[bp_idx][bp_sub_idx]=(curve[curve_original_idx,:3]-curve_exe[bp_exe_idx])
				###normal error direction
				R_temp=rotation_matrix_from_vectors(curve_exe_R[bp_exe_idx][:,-1],curve[curve_original_idx,3:])
				k_temp,theta_temp=R2rot(R_temp)
				if theta_temp!=0:
					error_bps_w[bp_idx][bp_sub_idx]=k_temp*theta_temp
				

		return error_bps_v,error_bps_w

	def update_error_direction(self,curve,p_bp,q_bp,error_bps_v,error_bps_w,gamma_v=0.8,gamma_w=0.1,extension=True):
		p_bp_new=copy.deepcopy(p_bp)
		q_bp_new=copy.deepcopy(q_bp)

		for bp_idx in range(len(p_bp)):
			for bp_sub_idx in range(len(p_bp[bp_idx])):
				
				if (bp_idx==0 and bp_sub_idx==0) or (bp_idx==len(p_bp)-1 and bp_sub_idx==len(p_bp[bp_idx])-1) and extension:
					step_v=2*gamma_v
					step_w=2*gamma_w
				else:
					step_v=1*gamma_v
					step_w=1*gamma_w
				#push toward error direction
				p_bp_new[bp_idx][bp_sub_idx]+=step_v*error_bps_v[bp_idx][bp_sub_idx]


				R_old=self.robot.fwd(q_bp[bp_idx][bp_sub_idx]).R

				theta_temp=np.linalg.norm(error_bps_w[bp_idx][bp_sub_idx])
				
				if theta_temp==0 or bp_sub_idx==1:	###if no angle error or it's movec mid point
					R_new=R_old
				else:
					k_temp=error_bps_w[bp_idx][bp_sub_idx]/theta_temp
					R_new=rot(k_temp,step_w*theta_temp)@R_old

				q_bp_new[bp_idx][bp_sub_idx]=car2js(self.robot,q_bp[bp_idx][bp_sub_idx],p_bp_new[bp_idx][bp_sub_idx],R_new)[0]

		return p_bp_new, q_bp_new


	def get_error_direction_dual(self,relative_path,p_bp1,q_bp1,p_bp2,q_bp2,relative_path_exe,relative_path_exe_R,curve_exe1,curve_exe_R1,curve_exe2,curve_exe_R2):
		###find points on curve_exe closest to all p_bp's, and closest point on blended trajectory to all p_bp's
		error_bps1=[]			#in robot2 tool frame
		error_bps2=[]			#in robot2 tool frame
		error_bps_v1=np.zeros((len(p_bp1),2,3))		#in global frame
		error_bps_w1=np.zeros((len(p_bp1),2,3))
		error_bps_v2=np.zeros((len(p_bp2),2,3))
		error_bps_w2=np.zeros((len(p_bp2),2,3))

		###get error direction for robot1
		for bp_idx in range(len(p_bp1)):
			for bp_sub_idx in range(len(p_bp1[bp_idx])):
				if bp_idx==0 and bp_sub_idx==0:
					curve_original_idx=0
					error_bp,bp_exe_idx=calc_error(relative_path[curve_original_idx,:3],relative_path_exe)
				elif bp_idx==len(p_bp1)-1 and bp_sub_idx==len(p_bp1[bp_idx])-1:
					curve_original_idx=len(relative_path)-1
					error_bp,bp_exe_idx=calc_error(relative_path[curve_original_idx,:3],relative_path_exe)
				else:
					_,bp_exe_idx=calc_error(p_bp1[bp_idx][bp_sub_idx],curve_exe1)							###find closest point on curve_exe to bp
					error_bp,curve_original_idx=calc_error(relative_path_exe[bp_exe_idx],relative_path[:,:3])	###find closest point on curve_original to curve_exe[bp]
				
				error_bps1.append(error_bp)
				###error direction in global frame (robot1 frame)
				error_bps_v1[bp_idx][bp_sub_idx]=(self.robot[-1].base_H[:3,:3]@curve_exe_R2[bp_exe_idx])@(relative_path[curve_original_idx,:3]-relative_path_exe[bp_exe_idx])
				###normal error direction
				R_temp=rotation_matrix_from_vectors(relative_path_exe_R[bp_exe_idx][:,-1],relative_path[curve_original_idx,3:])
				k_temp,theta_temp=R2rot(R_temp)
				###convert rotation axis from 2tool frame to robot1 base frame
				k_temp=(self.robot[-1].base_H[:3,:3]@curve_exe_R2[bp_exe_idx])@k_temp
				
				if theta_temp!=0:
					error_bps_w1[bp_idx][bp_sub_idx]=k_temp*theta_temp


		###get error direction for robot2
		for bp_idx in range(len(p_bp2)):
			for bp_sub_idx in range(len(p_bp2[bp_idx])):
				if bp_idx==0 and bp_sub_idx==0:
					curve_original_idx=0
					error_bp,bp_exe_idx=calc_error(relative_path[curve_original_idx,:3],relative_path_exe)
				elif bp_idx==len(p_bp2)-1 and bp_sub_idx==len(p_bp2[bp_idx])-1:
					curve_original_idx=len(relative_path)-1
					error_bp,bp_exe_idx=calc_error(relative_path[curve_original_idx,:3],relative_path_exe)
				else:
					_,bp_exe_idx=calc_error(p_bp2[bp_idx][bp_sub_idx],curve_exe2)							###find closest point on curve_exe to bp
					error_bp,curve_original_idx=calc_error(relative_path_exe[bp_exe_idx],relative_path[:,:3])	###find closest point on curve_original to curve_exe[bp]
				
				error_bps2.append(error_bp)
				###error direction in robot2 base frame, negate
				error_bps_v2[bp_idx][bp_sub_idx]=-curve_exe_R2[bp_exe_idx]@(relative_path[curve_original_idx,:3]-relative_path_exe[bp_exe_idx])
				###normal error direction
				R_temp=rotation_matrix_from_vectors(relative_path_exe_R[bp_exe_idx][:,-1],relative_path[curve_original_idx,3:]).T
				k_temp,theta_temp=R2rot(R_temp)
				###convert rotation axis from 2tool frame to robot2 base frame
				k_temp=curve_exe_R2[bp_exe_idx]@k_temp
				
				if theta_temp!=0:
					error_bps_w2[bp_idx][bp_sub_idx]=k_temp*theta_temp


		return error_bps_v1,error_bps_w1,error_bps_v2,error_bps_w2


	def update_error_direction_dual(self,relative_path,p_bp1,q_bp1,p_bp2,q_bp2,error_bps_v1,error_bps_w1,error_bps_v2,error_bps_w2,gamma_v=0.2,gamma_w=0.1,extension=True):
		
		for bp_idx in range(len(p_bp1)):
			for bp_sub_idx in range(len(p_bp1[bp_idx])):				
				if (bp_idx==0 and bp_sub_idx==0) or (bp_idx==len(p_bp1)-1 and bp_sub_idx==len(p_bp1[bp_idx])-1) and extension:
					step_v=2*gamma_v
					step_w=2*gamma_w
				else:
					step_v=1*gamma_v
					step_w=1*gamma_w
				#push toward error direction
				p_bp1[bp_idx][bp_sub_idx]+=step_v*error_bps_v1[bp_idx][bp_sub_idx]
				R_old=self.robot[0].fwd(q_bp1[bp_idx][bp_sub_idx]).R

				theta_temp=np.linalg.norm(error_bps_w1[bp_idx][bp_sub_idx])
				
				if theta_temp==0 or bp_sub_idx==1:	###if no angle error or it's movec mid point
					R_new=R_old
				else:
					k_temp=error_bps_w1[bp_idx][bp_sub_idx]/theta_temp
					R_new=rot(k_temp,step_w*theta_temp)@R_old

				q_bp1[bp_idx][bp_sub_idx]=car2js(self.robot[0],q_bp1[bp_idx][bp_sub_idx],p_bp1[bp_idx][bp_sub_idx],R_new)[0]

		for bp_idx in range(len(p_bp2)):
			for bp_sub_idx in range(len(p_bp2[bp_idx])):				
				if (bp_idx==0 and bp_sub_idx==0) or (bp_idx==len(p_bp1)-1 and bp_sub_idx==len(p_bp1[bp_idx])-1) and extension:
					step_v=2*gamma_v
					step_w=2*gamma_w
				else:
					step_v=1*gamma_v
					step_w=1*gamma_w
				#push toward error direction
				p_bp2[bp_idx][bp_sub_idx]+=step_v*error_bps_v2[bp_idx][bp_sub_idx]
				R_old=self.robot[1].fwd(q_bp2[bp_idx][bp_sub_idx]).R

				theta_temp=np.linalg.norm(error_bps_w2[bp_idx][bp_sub_idx])
				
				if theta_temp==0 or bp_sub_idx==1:	###if no angle error or it's movec mid point
					R_new=R_old
				else:
					k_temp=error_bps_w2[bp_idx][bp_sub_idx]/theta_temp
					R_new=rot(k_temp,step_w*theta_temp)@R_old

				q_bp2[bp_idx][bp_sub_idx]=car2js(self.robot[1],q_bp2[bp_idx][bp_sub_idx],p_bp2[bp_idx][bp_sub_idx],R_new)[0]


		return p_bp1, q_bp1, p_bp2, q_bp2


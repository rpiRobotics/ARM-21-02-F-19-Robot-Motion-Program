import numpy as np
from general_robotics_toolbox import *
from pandas import read_csv
import sys
from io import StringIO

# sys.path.append('../abb_motion_program_exec')
from abb_motion_program_exec import *
sys.path.append('../toolbox')
from robots_def import *
from error_check import *
from MotionSend import *
from lambda_calc import *
from blending import *

class ilc_toolbox(object):
	def __init__(self,robot,primitives,base2_R=np.eye(3),base2_p=np.zeros(3)):
		#robot: single robot or tuple
		#primitives: series of primitives or list of 2 
		self.robot=robot
		self.primitives=primitives
		self.base2_R=base2_R
		self.base2_p=base2_p

	
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


	def update_bp_xyz(self,p_bp,q_bp,de_dp,max_error,breakpoint_interp_2tweak_indices,alpha=0.5):
		###p_bp:								xyz of breakpoints
		###q_bp:								joint configs of breakpoints
		###de_dp；								gradient of error and each breakpoints
		###max_error:							worst case error value
		###breakpoint_interp_2tweak_indices:	closest N breakpoints
		###alpha:								stepsize

		point_adjustment=-alpha*np.linalg.pinv(de_dp)*max_error
		# print(max_error,point_adjustment)
		idx=0
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			for bp_sub_idx in range(len(p_bp[m])):

				p_bp[m][bp_sub_idx]+=point_adjustment[0][3*idx:3*(idx+1)]
				q_bp[m][bp_sub_idx]=car2js(self.robot,q_bp[m][bp_sub_idx],p_bp[m][bp_sub_idx],self.robot.fwd(q_bp[m][bp_sub_idx]).R)[0]
				idx+=1
		return p_bp, q_bp


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
				###error direction
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

				q_bp_new[bp_idx][bp_sub_idx]=car2js(self.robot,q_bp[bp_idx][bp_sub_idx],p_bp[bp_idx][bp_sub_idx],R_new)[0]

		return p_bp_new, q_bp_new
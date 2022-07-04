import numpy as np
from general_robotics_toolbox import *
from pandas import read_csv
import sys
from io import StringIO

# sys.path.append('../abb_motion_program_exec')
from abb_motion_program_exec_client import *
sys.path.append('../toolbox')
from robots_def import *
from error_check import *
from MotionSend import *
from lambda_calc import *
from blending import *

class ilc_toolbox(object):
	def __init__(self,robot,primitives,base2_R=np.eye(3),base2_p=np.zeros(3)):
		self.robot=robot
		self.primitives=primitives
		self.base2_R=base2_R
		self.base2_p=base2_p
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

		###TODO:ADD MOVEC SUPPORT
		de_dp=[]    #de_dp1q1,de_dp1q2,...,de_dp3q6
		de_ori_dp=[]
		delta=0.1 	#mm

		###len(primitives)==len(breakpoints)==len(breakpoints_blended)==len(points_list)
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			for n in range(3): #3DOF, xyz
				q_bp_temp=np.array(copy.deepcopy(q_bp))
				p_bp_temp=copy.deepcopy(p_bp)
				p_bp_temp[m][0][n]+=delta

				q_bp_temp[m][0]=car2js(self.robot,q_bp[m][0],np.array(p_bp_temp[m][0]),self.robot.fwd(q_bp[m][0]).R)[0]###TODO:ADD MOVEC SUPPORT

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

		return de_dp
	
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

		###len(primitives)==len(breakpoints)==len(breakpoints_blended)==len(points_list)
		for m in breakpoint_interp_2tweak_indices:  #3 breakpoints
			for n in range(3): #3DOF, xyz
				q_bp_temp=np.array(copy.deepcopy(q_bp))
				p_bp_temp=copy.deepcopy(p_bp)
				p_bp_temp[m][0][n]+=delta

				q_bp_temp[m][0]=car2js(self.robot,q_bp[m][0],np.array(p_bp_temp[m][0]),self.robot.fwd(q_bp[m][0]).R)[0]###TODO:ADD MOVEC SUPPORT

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

				# curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_cart_from_primitive(curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[i] for i in short_version],self.robot,speed)
				curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[i] for i in short_version],self.robot,zone=10)
				
				curve_blended_new=copy.deepcopy(curve_blended)

				

				curve_blended_new[start_idx:end_idx]=curve_blended_temp[start_idx-breakpoints_blended[short_version[0]]:len(curve_blended_temp)-(breakpoints_blended[short_version[-1]]+1-end_idx)]

				###calculate relative gradient
				worst_case_point_shift=curve_blended_new[max_error_curve_blended_idx]-curve_blended[max_error_curve_blended_idx]

				###get new error - prev error
				de=np.linalg.norm(worst_point_pose.p+worst_case_point_shift-closest_p)-np.linalg.norm(worst_point_pose.p-closest_p)

				de_dp.append(de/delta)

		de_dp=np.reshape(de_dp,(-1,1))

		return de_dp

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
		robot2_worst_pose_global=self.robot[-1].fwd(worst_point_joints[-1],self.base2_R,self.base2_p)
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

		point_adjustment=-alpha*np.linalg.pinv(de_dp)*max_error

		for i in range(len(breakpoint_interp_2tweak_indices)):  #3 breakpoints
			p_bp[breakpoint_interp_2tweak_indices[i]][0]+=point_adjustment[0][3*i:3*(i+1)]
			###TODO:ADD MOVEC SUPPORT
			q_bp[breakpoint_interp_2tweak_indices[i]][0]=car2js(self.robot,q_bp[breakpoint_interp_2tweak_indices[i]][0],p_bp[breakpoint_interp_2tweak_indices[i]][0],self.robot.fwd(q_bp[breakpoint_interp_2tweak_indices[i]][0]).R)[0]

		return p_bp, q_bp

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
				q_bp_temp=copy.deepcopy(q_bp)
				q_bp_temp[m][0][n]+=delta		###ADD MOVEC SUPPORT
				
				#restore new trajectory, only for adjusted breakpoint, 1-bp change requires traj interp from 5 bp
				short_version=range(max(m-2,0),min(m+2,len(breakpoints_blended)-1))
				curve_interp_temp, curve_R_interp_temp, curve_js_interp_temp, breakpoints_blended_temp=form_traj_from_bp(q_bp_temp[short_version],[self.primitives[i] for i in short_version],self.robot)

				curve_js_blended_temp,curve_blended_temp,curve_R_blended_temp=blend_js_from_primitive(curve_interp_temp, curve_js_interp_temp, breakpoints_blended_temp, [self.primitives[i] for i in short_version],self.robot,zone=10)
				
				curve_blended_new=copy.deepcopy(curve_blended)
				start_idx=int((breakpoints_blended[short_version[0]]+breakpoints_blended[short_version[1]])/2)
				end_idx=int((breakpoints_blended[short_version[-1]]+breakpoints_blended[short_version[-2]])/2)

				curve_blended_new[start_idx:end_idx]=curve_blended_temp[start_idx-breakpoints_blended[short_version[0]]:-1-(breakpoints_blended[short_version[-1]]-end_idx)]

				###calculate relative gradient, xyz
				worst_case_point_shift=curve_blended_new[max_error_curve_blended_idx]-curve_blended[max_error_curve_blended_idx]
				###get new error
				de=np.linalg.norm(worst_point_pose.p+worst_case_point_shift-closest_p)-np.linalg.norm(worst_point_pose.p-closest_p)

				###calculate relative gradient, ori
				worst_case_R_shift=curve_R_blended_temp[max_error_curve_blended_idx]@curve_R_blended[max_error_curve_blended_idx].T
				de_ori=get_angle(worst_case_R_shift@worst_point_pose.R,closest_N)-get_angle(worst_point_pose.R,closest_N)

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
			q_bp[breakpoint_interp_2tweak_indices[i]]+=bp_q_adjustment[0][6*i:6*(i+1)]
			p_bp[breakpoint_interp_2tweak_indices[i]]=self.robot.fwd(q_bp[breakpoint_interp_2tweak_indices[i]]).p

		return p_bp, q_bp

	def get_error_bp(self,p_bp,q_bp,curve_exe,curve_exe_R,curve_bp,curve_R_bp):
		###p_bp:								xyz of breakpoints, u
		###q_bp:								joint configs of breakpoints, u
		###curve_exe:							execution curve
		###curve_exe_R:							execution curve R
		###curve_bp:							"breakpoints" on original curve, y_d
		###curve_R_bp:							"breakpoints" on original curve R, y_d


		exe_bp_idx=[]
		ep=[]
		eR=[]
		###calculate error excluding start & end
		for i in range(1,len(p_bp)-1):
			###get output y index by finding closest on curve_exe
			exe_bp_idx.append(calc_error(p_bp[i][0],curve_exe)[1])
			###find error y-y_d
			ep.append(curve_exe[exe_bp_idx[-1]]-curve_bp[i][0])
			# eR.append(curve_exe_R[exe_bp_idx[-1]]@self.robot.fwd(q_bp[i][0]).R.T)

		return np.array(ep), np.array(eR), curve_exe[exe_bp_idx],curve_exe_R[exe_bp_idx]

	def get_error_bp2(self,p_bp,q_bp,curve_exe,curve_exe_R,curve,curve_R):
		###p_bp:								xyz of breakpoints, u
		###q_bp:								joint configs of breakpoints, u
		###curve_exe:							execution curve
		###curve_exe_R:							execution curve R
		###curve:								original curve
		###curve:								original curve_R


		exe_bp_idx=[]
		ep=[]
		eR=[]
		###calculate error excluding start & end
		for i in range(1,len(p_bp)-1):
			###get output y index by finding closest on curve_exe
			exe_bp_idx.append(calc_error(p_bp[i][0],curve_exe)[1])
			###get y_d by finding closest on original curve
			y_d=curve[calc_error(curve_exe[exe_bp_idx[-1]],curve)[1]]

			ep.append(curve_exe[exe_bp_idx[-1]]-y_d)
			# eR.append(curve_exe_R[exe_bp_idx[-1]]@self.robot.fwd(q_bp[i][0]).R.T)

		return np.array(ep), np.array(eR), curve_exe[exe_bp_idx],curve_exe_R[exe_bp_idx]



	def update_bp_ilc(self,p_bp,q_bp,exe_bp_p,exe_bp_R,exe_bp_p_new, exe_bp_R_new,alpha1=0.5,alpha2=0.5):
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
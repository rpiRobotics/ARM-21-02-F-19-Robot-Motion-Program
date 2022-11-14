import numpy as np
from general_robotics_toolbox import *
from pandas import read_csv
import sys
from abb_motion_program_exec_client import *
from robots_def import *
from error_check import *
from toolbox_circular_fit import *
from lambda_calc import *
from dual_arm import *


class MotionSend(object):
	def __init__(self,url='http://127.0.0.1:80') -> None:
		self.client = MotionProgramExecClient(base_url=url)

	def moveL_target(self,robot,q,point):
		quat=R2q(robot.fwd(q).R)
		cf=quadrant(q,robot)
		robt = robtarget([point[0], point[1], point[2]], [ quat[0], quat[1], quat[2], quat[3]], confdata(cf[0],cf[1],cf[2],cf[3]),[9E+09]*6)
		return robt

	def moveL_target_relative(self,robot1,robot2,q1,q2,point):
		pose1_now=robot1.fwd(q1)
		pose2_world_now=robot2.fwd(q2,world=True)
		relative_R=pose2_world_now.R.T@pose1_now.R

		quat=R2q(relative_R)
		cf=quadrant(q1,robot1)
		robt = robtarget([point[0], point[1], point[2]], [ quat[0], quat[1], quat[2], quat[3]], confdata(cf[0],cf[1],cf[2],cf[3]),[9E+09]*6)
		return robt
	
	def moveC_target(self,robot,q1,q2,point1,point2):
		quat1=R2q(robot.fwd(q1).R)
		cf1=quadrant(q1,robot)
		quat2=R2q(robot.fwd(q2).R)
		cf2=quadrant(q2,robot)
		robt1 = robtarget([point1[0], point1[1], point1[2]], [ quat1[0], quat1[1], quat1[2], quat1[3]], confdata(cf1[0],cf1[1],cf1[2],cf1[3]),[0]*6)
		robt2 = robtarget([point2[0], point2[1], point2[2]], [ quat2[0], quat2[1], quat2[2], quat2[3]], confdata(cf2[0],cf2[1],cf2[2],cf2[3]),[0]*6)
		return robt1, robt2

	def moveJ_target(self,q):
		q = np.rad2deg(q)
		jointt = jointtarget([q[0],q[1],q[2],q[3],q[4],q[5]],[0]*6)
		return jointt

	def jog_joint_multimove(self,q1,q2):
		mp1 = MotionProgram()
		mp2 = MotionProgram()
		mp1.MoveAbsJ(self.moveJ_target(q1),v200,fine)
		mp2.MoveAbsJ(self.moveJ_target(q2),v200,fine)
		self.client.execute_multimove_motion_program([mp1,mp2])

	def exec_motions(self,robot,primitives,breakpoints,p_bp,q_bp,speed,zone):

		mp = MotionProgram(tool=tooldata(True,pose(robot.p_tool,R2q(robot.R_tool)),loaddata(1,[0,0,0.001],[1,0,0,0],0,0,0)))
		###change cirpath mode
		mp.CirPathMode(CirPathModeSwitch.ObjectFrame)

		
		for i in range(len(primitives)):
			if 'movel' in primitives[i]:

				robt = self.moveL_target(robot,q_bp[i][0],p_bp[i][0])
				if type(speed) is list:
					if type(zone) is list:
						mp.MoveL(robt,speed[i],zone[i])
					else:
						mp.MoveL(robt,speed[i],zone)
				else:
					if type(zone) is list:
						mp.MoveL(robt,speed,zone[i])
					else:
						mp.MoveL(robt,speed,zone)
				

			elif 'movec' in primitives[i]:
				robt1, robt2 = self.moveC_target(robot,q_bp[i][0],q_bp[i][1],p_bp[i][0],p_bp[i][1])
				if type(speed) is list:
					if type(zone) is list:
						mp.MoveC(robt1,robt2,speed[i],zone[i])
					else:
						mp.MoveC(robt1,robt2,speed[i],zone)
				else:
					if type(zone) is list:
						mp.MoveC(robt1,robt2,speed,zone[i])
					else:
						mp.MoveC(robt1,robt2,speed,zone)

			elif 'movej' in primitives[i]:
				robt = self.moveL_target(robot,q_bp[i][0],p_bp[i][0])
				if type(speed) is list:
					if type(zone) is list:
						mp.MoveJ(robt,speed[i],zone[i])
					else:
						mp.MoveJ(robt,speed[i],zone)
				else:
					if type(zone) is list:
						mp.MoveJ(robt,speed,zone[i])
					else:
						mp.MoveJ(robt,speed,zone)

			else: # moveabsj
				jointt = self.moveJ_target(q_bp[i][0])
				if i==0:
					mp.MoveAbsJ(jointt,v500,fine)
					mp.WaitTime(1)
					mp.MoveAbsJ(jointt,v500,fine)
					mp.WaitTime(0.1)
				else:
					if type(speed) is list:
						if type(zone) is list:
							mp.MoveAbsJ(jointt,speed[i],zone[i])
						else:
							mp.MoveAbsJ(jointt,speed[i],zone)
					else:
						if type(zone) is list:
							mp.MoveAbsJ(jointt,speed,zone[i])
						else:
							mp.MoveAbsJ(jointt,speed,zone)
		###add sleep at the end to wait for train_data transmission
		mp.WaitTime(0.1)
		
		# print(mp.get_program_rapid())
		log_results = self.client.execute_motion_program(mp)
		return log_results

	def exe_from_file(self,robot,filename,speed,zone):
		breakpoints,primitives, p_bp,q_bp=self.extract_data_from_cmd(filename)
		return self.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,speed,zone)

	def exec_motions_multimove(self,robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,speed1,speed2,zone1,zone2):
		###dynamic speed2
		
		mp1 = MotionProgram(tool=tooldata(True,pose(robot1.p_tool,R2q(robot1.R_tool)),loaddata(1,[0,0,0.001],[1,0,0,0],0,0,0)))
		mp2 = MotionProgram(tool=tooldata(True,pose(robot2.p_tool,R2q(robot2.R_tool)),loaddata(1,[0,0,0.001],[1,0,0,0],0,0,0)))

		###change cirpath mode
		mp1.CirPathMode(CirPathModeSwitch.ObjectFrame)
		mp2.CirPathMode(CirPathModeSwitch.ObjectFrame)

		
		for i in range(len(primitives1)):

			if 'movel' in primitives1[i]:
				robt = self.moveL_target(robot1,q_bp1[i][0],p_bp1[i][0])
				if type(speed1) is list:
					if type(zone1) is list:
						mp1.MoveL(robt,speed1[i],zone1[i])
					else:
						mp1.MoveL(robt,speed1[i],zone1)
				else:
					if type(zone1) is list:
						mp1.MoveL(robt,speed1,zone1[i])
					else:
						mp1.MoveL(robt,speed1,zone1)


			elif 'movec' in primitives1[i]:
				robt1, robt2 = self.moveC_target(robot1,q_bp1[i][0],q_bp1[i][1],p_bp1[i][0],p_bp1[i][1])
				if type(speed1) is list:
					if type(zone1) is list:
						mp1.MoveC(robt1,robt2,speed1[i],zone1[i])
					else:
						mp1.MoveC(robt1,robt2,speed1[i],zone1)
				else:
					if type(zone1) is list:
						mp1.MoveC(robt1,robt2,speed1,zone1[i])
					else:
						mp1.MoveC(robt1,robt2,speed1,zone1)

			elif 'movej' in primitives1[i]:
				robt = self.moveL_target(robot1,q_bp1[i][0],p_bp1[i][0])
				mp1.MoveJ(robt,speed1,zone1)

			else: # moveabsj
				jointt = self.moveJ_target(q_bp1[i][0])
				if i==0:
					mp1.MoveAbsJ(jointt,v500,fine)
					mp1.WaitTime(1)
					mp1.MoveAbsJ(jointt,v500,fine)
					mp1.WaitTime(0.1)
				else:
					if type(speed1) is list:
						if type(zone1) is list:
							mp1.MoveAbsJ(jointt,speed1[i],zone1[i])
						else:
							mp1.MoveAbsJ(jointt,speed1[i],zone1)
					else:
						if type(zone1) is list:
							mp1.MoveAbsJ(jointt,speed1,zone1[i])
						else:
							mp1.MoveAbsJ(jointt,speed1,zone1)

		for i in range(len(primitives2)):
			if 'movel' in primitives2[i]:
				robt = self.moveL_target(robot2,q_bp2[i][0],p_bp2[i][0])
				if type(speed2) is list:
					if type(zone2) is list:
						mp2.MoveL(robt,speed2[i],zone2[i])
					else:
						mp2.MoveL(robt,speed2[i],zone2)
				else:
					if type(zone2) is list:
						mp2.MoveL(robt,speed2,zone2[i])
					else:
						mp2.MoveL(robt,speed2,zone2)


			elif 'movec' in primitives2[i]:
				robt1, robt2 = self.moveC_target(robot2,q_bp2[i][0],q_bp2[i][1],p_bp2[i][0],p_bp2[i][1])
				if type(speed2) is list:
					if type(zone2) is list:
						mp2.MoveC(robt1,robt2,speed2[i],zone2[i])
					else:
						mp2.MoveC(robt1,robt2,speed2[i],zone2)
				else:
					if type(zone2) is list:
						mp2.MoveC(robt1,robt2,speed2,zone2[i])
					else:
						mp2.MoveC(robt1,robt2,speed2,zone2)

			elif 'movej' in primitives2[i]:
				robt = self.moveL_target(robot2,q_bp2[i][0],p_bp2[i][0])
				if type(speed2) is list:
					mp2.MoveJ(robt,speed2[i],zone2)
				else:
					mp2.MoveJ(robt,speed2,zone2)

			else: # moveabsj
				jointt = self.moveJ_target(q_bp2[i][0])
				if i==0:
					mp2.MoveAbsJ(jointt,v500,fine)
					mp2.WaitTime(1)
					mp2.MoveAbsJ(jointt,v500,fine)
					mp2.WaitTime(0.1)
				else:
					if type(speed2) is list:
						if type(zone2) is list:
							mp2.MoveAbsJ(jointt,speed2[i],zone2[i])
						else:
							mp2.MoveAbsJ(jointt,speed2[i],zone2)
					else:
						if type(zone2) is list:
							mp2.MoveAbsJ(jointt,speed2,zone2[i])
						else:
							mp2.MoveAbsJ(jointt,speed2,zone2)

		###add sleep at the end to wait for train_data transmission
		mp1.WaitTime(0.1)
		mp2.WaitTime(0.1)
		
		# print(mp1.get_program_rapid())
		# print(mp2.get_program_rapid())
		return self.client.execute_multimove_motion_program([mp1,mp2])

		

	def exec_motions_multimove_relative(self,robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,speed1,speed2,zone1,zone2):

		wobj = wobjdata(False,False,"ROB_2",pose([0,0,0],[1,0,0,0]),pose(robot2.p_tool,R2q(robot2.R_tool)))

		###dynamic speed2
		mp1 = MotionProgram(tool=tooldata(True,pose(robot1.p_tool,R2q(robot1.R_tool)),loaddata(1,[0,0,0.001],[1,0,0,0],0,0,0)),wobj=wobj)
		mp2 = MotionProgram(tool=tooldata(True,pose(robot2.p_tool,R2q(robot2.R_tool)),loaddata(1,[0,0,0.001],[1,0,0,0],0,0,0)))

		###change cirpath mode
		# mp1.CirPathMode(CirPathModeSwitch.ObjectFrame)
		# mp2.CirPathMode(CirPathModeSwitch.ObjectFrame)
		
		for i in range(len(primitives1)):

			if 'movel' in primitives1[i]:
				robt = self.moveL_target_relative(robot1,robot2,q_bp1[i][0],q_bp2[i][0],p_bp1[i][0])
				if type(speed1) is list:
					if type(zone1) is list:
						mp1.MoveL(robt,speed1[i],zone1[i])
					else:
						mp1.MoveL(robt,speed1[i],zone1)
				else:
					if type(zone1) is list:
						mp1.MoveL(robt,speed1,zone1[i])
					else:
						mp1.MoveL(robt,speed1,zone1)


			elif 'movec' in primitives1[i]:
				robt1, robt2 = self.moveC_target_relative(robot1,robot2,q_bp1[i][0],q_bp1[i][1],p_bp1[i][0],p_bp1[i][1])
				if type(speed1) is list:
					if type(zone1) is list:
						mp1.MoveC(robt1,robt2,speed1[i],zone1[i])
					else:
						mp1.MoveC(robt1,robt2,speed1[i],zone1)
				else:
					if type(zone1) is list:
						mp1.MoveC(robt1,robt2,speed1,zone1[i])
					else:
						mp1.MoveC(robt1,robt2,speed1,zone1)

			elif 'movej' in primitives1[i]:
				robt = self.moveL_target(robot1,q_bp1[i][0],p_bp1[i][0])
				mp1.MoveJ(robt,speed1,zone1)

			else: # moveabsj
				jointt = self.moveJ_target(q_bp1[i][0])
				if i==0:
					mp1.MoveAbsJ(jointt,v500,fine)
					mp1.WaitTime(1)
					mp1.MoveAbsJ(jointt,v500,fine)
					mp1.WaitTime(0.1)
				else:
					if type(speed1) is list:
						if type(zone1) is list:
							mp1.MoveAbsJ(jointt,speed1[i],zone1[i])
						else:
							mp1.MoveAbsJ(jointt,speed1[i],zone1)
					else:
						if type(zone1) is list:
							mp1.MoveAbsJ(jointt,speed1,zone1[i])
						else:
							mp1.MoveAbsJ(jointt,speed1,zone1)

		for i in range(len(primitives2)):
			if 'movel' in primitives2[i]:
				robt = self.moveL_target(robot2,q_bp2[i][0],p_bp2[i][0])
				if type(speed2) is list:
					if type(zone2) is list:
						mp2.MoveL(robt,speed2[i],zone2[i])
					else:
						mp2.MoveL(robt,speed2[i],zone2)
				else:
					if type(zone2) is list:
						mp2.MoveL(robt,speed2,zone2[i])
					else:
						mp2.MoveL(robt,speed2,zone2)


			elif 'movec' in primitives2[i]:
				robt1, robt2 = self.moveC_target(robot2,q_bp2[i][0],q_bp2[i][1],p_bp2[i][0],p_bp2[i][1])
				if type(speed2) is list:
					if type(zone2) is list:
						mp2.MoveC(robt1,robt2,speed2[i],zone2[i])
					else:
						mp2.MoveC(robt1,robt2,speed2[i],zone2)
				else:
					if type(zone2) is list:
						mp2.MoveC(robt1,robt2,speed2,zone2[i])
					else:
						mp2.MoveC(robt1,robt2,speed2,zone2)

			elif 'movej' in primitives2[i]:
				robt = self.moveL_target(robot2,q_bp2[i][0],p_bp2[i][0])
				if type(speed2) is list:
					mp2.MoveJ(robt,speed2[i],zone2)
				else:
					mp2.MoveJ(robt,speed2,zone2)

			else: # moveabsj
				jointt = self.moveJ_target(q_bp2[i][0])
				if i==0:
					mp2.MoveAbsJ(jointt,v500,fine)
					mp2.WaitTime(1)
					mp2.MoveAbsJ(jointt,v500,fine)
					mp2.WaitTime(0.1)
				else:
					if type(speed2) is list:
						if type(zone2) is list:
							mp2.MoveAbsJ(jointt,speed2[i],zone2[i])
						else:
							mp2.MoveAbsJ(jointt,speed2[i],zone2)
					else:
						if type(zone2) is list:
							mp2.MoveAbsJ(jointt,speed2,zone2[i])
						else:
							mp2.MoveAbsJ(jointt,speed2,zone2)

		###add sleep at the end to wait for train_data transmission
		mp1.WaitTime(0.1)
		mp2.WaitTime(0.1)
		
		# print(mp1.get_program_rapid(module_name="TROB1_MODULE",sync_move=True))
		# print(mp2.get_program_rapid(module_name="TROB2_MODULE", sync_move=True))
		return self.client.execute_multimove_motion_program([mp1,mp2])


	def extend(self,robot,q_bp,primitives,breakpoints,points_list,extension_start=100,extension_end=100):
		###initial point extension
		pose_start=robot.fwd(q_bp[0][-1])
		p_start=pose_start.p
		R_start=pose_start.R
		pose_end=robot.fwd(q_bp[1][-1])
		p_end=pose_end.p
		R_end=pose_end.R
		if 'movel' in primitives[1]:
			#find new start point
			slope_p=p_end-p_start
			slope_p=slope_p/np.linalg.norm(slope_p)
			p_start_new=p_start-extension_start*slope_p        ###extend 5cm backward

			#find new start orientation
			k,theta=R2rot(R_end@R_start.T)
			theta_new=-extension_start*theta/np.linalg.norm(p_end-p_start)
			R_start_new=rot(k,theta_new)@R_start

			#solve invkin for initial point
			points_list[0][0]=p_start_new
			q_bp[0][0]=car2js(robot,q_bp[0][0],p_start_new,R_start_new)[0]

		elif 'movec' in primitives[1]:
			#define circle first
			pose_mid=robot.fwd(q_bp[1][0])
			p_mid=pose_mid.p
			R_mid=pose_mid.R

			center, radius=circle_from_3point(p_start,p_end,p_mid)

			#find desired rotation angle
			angle=extension_start/radius

			#find new start point
			plane_N=np.cross(p_end-center,p_start-center)
			plane_N=plane_N/np.linalg.norm(plane_N)
			R_temp=rot(plane_N,angle)
			p_start_new=center+R_temp@(p_start-center)

			#modify mid point to be in the middle of new start and old end (to avoid RS circle uncertain error)
			modified_bp=arc_from_3point(p_start_new,p_end,p_mid,N=3)
			points_list[1][0]=modified_bp[1]

			#find new start orientation
			k,theta=R2rot(R_end@R_start.T)
			theta_new=-extension_start*theta/np.linalg.norm(p_end-p_start)
			R_start_new=rot(k,theta_new)@R_start

			#solve invkin for initial point
			points_list[0][0]=p_start_new
			q_bp[0][0]=car2js(robot,q_bp[0][0],p_start_new,R_start_new)[0]


		else:
			#find new start point
			J_start=robot.jacobian(q_bp[0][0])
			qdot=q_bp[0][0]-q_bp[1][0]
			v=np.linalg.norm(J_start[3:,:]@qdot)
			t=extension_start/v
			q_bp[0][0]=q_bp[0][0]+qdot*t
			points_list[0][0]=robot.fwd(q_bp[0][0]).p

		###end point extension
		pose_start=robot.fwd(q_bp[-2][-1])
		p_start=pose_start.p
		R_start=pose_start.R
		pose_end=robot.fwd(q_bp[-1][-1])
		p_end=pose_end.p
		R_end=pose_end.R

		if 'movel' in primitives[-1]:
			#find new end point
			slope_p=(p_end-p_start)/np.linalg.norm(p_end-p_start)
			p_end_new=p_end+extension_end*slope_p        ###extend 5cm backward

			#find new end orientation
			k,theta=R2rot(R_end@R_start.T)
			slope_theta=theta/np.linalg.norm(p_end-p_start)
			R_end_new=rot(k,extension_end*slope_theta)@R_end

			#solve invkin for end point
			q_bp[-1][0]=car2js(robot,q_bp[-1][0],p_end_new,R_end_new)[0]
			points_list[-1][0]=p_end_new


		elif  'movec' in primitives[-1]:
			#define circle first
			pose_mid=robot.fwd(q_bp[-1][0])
			p_mid=pose_mid.p
			R_mid=pose_mid.R
			center, radius=circle_from_3point(p_start,p_end,p_mid)

			#find desired rotation angle
			angle=extension_end/radius

			#find new end point
			plane_N=np.cross(p_start-center,p_end-center)
			plane_N=plane_N/np.linalg.norm(plane_N)
			R_temp=rot(plane_N,angle)
			p_end_new=center+R_temp@(p_end-center)

			#modify mid point to be in the middle of new end and old start (to avoid RS circle uncertain error)
			modified_bp=arc_from_3point(p_start,p_end_new,p_mid,N=3)
			points_list[-1][0]=modified_bp[1]

			#find new end orientation
			k,theta=R2rot(R_end@R_start.T)
			theta_new=extension_end*theta/np.linalg.norm(p_end-p_start)
			R_end_new=rot(k,theta_new)@R_end

			#solve invkin for end point
			q_bp[-1][-1]=car2js(robot,q_bp[-1][-1],p_end_new,R_end_new)[0]
			points_list[-1][-1]=p_end_new   #midpoint not changed

		else:
			#find new end point
			J_end=robot.jacobian(q_bp[-1][0])
			qdot=q_bp[-1][0]-q_bp[-2][0]
			v=np.linalg.norm(J_end[3:,:]@qdot)
			t=extension_end/v
			
			q_bp[-1][0]=q_bp[-1][-1]+qdot*t
			points_list[-1][0]=robot.fwd(q_bp[-1][-1]).p

		return points_list,q_bp

	def extend_dual(self,robot1,p_bp1,q_bp1,primitives1,robot2,p_bp2,q_bp2,primitives2,breakpoints,extension_start2=100,extension_end2=100):
		#extend porpotionally
		d1_start=np.linalg.norm(p_bp1[1][-1]-p_bp1[0][-1])
		d2_start=np.linalg.norm(p_bp2[1][-1]-p_bp2[0][-1])
		d1_end=np.linalg.norm(p_bp1[-1][-1]-p_bp1[-2][-1])
		d2_end=np.linalg.norm(p_bp2[-1][-1]-p_bp2[-2][-1])

		p_bp1,q_bp1=self.extend(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_start2*d1_start/d2_start,extension_end=extension_end2*d1_end/d2_end)
		p_bp2,q_bp2=self.extend(robot2,q_bp2,primitives2,breakpoints,p_bp2,extension_start=extension_start2,extension_end=extension_end2)

		return p_bp1,q_bp1,p_bp2,q_bp2
	def extract_data_from_cmd(self,filename):
		data = read_csv(filename)
		breakpoints=np.array(data['breakpoints'].tolist())
		primitives=data['primitives'].tolist()
		points=data['p_bp'].tolist()
		qs=data['q_bp'].tolist()

		p_bp=[]
		q_bp=[]
		for i in range(len(breakpoints)):
			if 'movel' in primitives[i]:
				point=extract_points(primitives[i],points[i])
				p_bp.append([point])
				q=extract_points(primitives[i],qs[i])
				q_bp.append([q])


			elif 'movec' in primitives[i]:
				point1,point2=extract_points(primitives[i],points[i])
				p_bp.append([point1,point2])
				q1,q2=extract_points(primitives[i],qs[i])
				q_bp.append([q1,q2])

			else:
				point=extract_points(primitives[i],points[i])
				p_bp.append([point])
				q=extract_points(primitives[i],qs[i])
				q_bp.append([q])

		return breakpoints,primitives, p_bp,q_bp
	def write_data_to_cmd(self,filename,breakpoints,primitives, p_bp,q_bp):
		p_bp_new=[]
		q_bp_new=[]
		for i in range(len(breakpoints)):
			if len(p_bp[i])==2:
				p_bp_new.append([np.array(p_bp[i][0]),np.array(p_bp[i][1])])
				q_bp_new.append([np.array(q_bp[i][0]),np.array(q_bp[i][1])])
			else:
				p_bp_new.append([np.array(p_bp[i][0])])
				q_bp_new.append([np.array(q_bp[i][0])])
		df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'p_bp':p_bp_new,'q_bp':q_bp_new})
		df.to_csv(filename,header=True,index=False)


	def exe_from_file_multimove(self,filename1,filename2,speed1,speed2,zone1,zone2):
		breakpoints1,primitives1,p_bp1,q_bp1=ms.extract_data_from_cmd(filename1)
		breakpoints2,primitives2,p_bp2,q_bp2=ms.extract_data_from_cmd(filename2)

		return ms.exec_motions_multimove(breakpoints1,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,speed1,speed2,zone1,zone2)


	def logged_data_analysis(self,robot,log_results,realrobot=True):
		cmd_num=log_results.data[:,1]
		#find closest to 5 cmd_num
		idx = np.absolute(cmd_num-5).argmin()
		start_idx=np.where(cmd_num==cmd_num[idx])[0][0]
		curve_exe_js=np.radians(log_results.data[start_idx:,2:])
		timestamp=log_results.data[start_idx:,0]
		###filter noise
		timestamp, curve_exe_js=lfilter(timestamp, curve_exe_js)

		speed=[0]
		lam=[0]
		curve_exe=[]
		curve_exe_R=[]
		for i in range(len(curve_exe_js)):
			robot_pose=robot.fwd(curve_exe_js[i],qlim_override=True)
			curve_exe.append(robot_pose.p)
			curve_exe_R.append(robot_pose.R)
			if i>0:
				lam.append(lam[-1]+np.linalg.norm(curve_exe[i]-curve_exe[i-1]))
			try:
				if timestamp[i-1]!=timestamp[i] and np.linalg.norm(curve_exe_js[i-1]-curve_exe_js[i])!=0:
					speed.append(np.linalg.norm(curve_exe[-1]-curve_exe[-2])/(timestamp[i]-timestamp[i-1]))
				else:
					speed.append(speed[-1])      
			except IndexError:
				pass

		speed=moving_average(speed,padding=True)
		return np.array(lam), np.array(curve_exe), np.array(curve_exe_R),np.array(curve_exe_js), np.array(speed), timestamp-timestamp[0]

	def logged_data_analysis_multimove(self,log_results,robot1,robot2,realrobot=True):
		cmd_num=log_results.data[:,1]

		idx = np.absolute(cmd_num-5).argmin()
		start_idx=np.where(cmd_num==cmd_num[idx])[0][0]

		curve_exe_js1=np.radians(log_results.data[start_idx:,2:8])
		curve_exe_js2=np.radians(log_results.data[start_idx:,8:])
		timestamp=log_results.data[start_idx:,0]


		if realrobot:
			
			timestamp, curve_exe_js_all=lfilter(timestamp, np.hstack((curve_exe_js1,curve_exe_js2)))
			curve_exe_js1=curve_exe_js_all[:,:6]
			curve_exe_js2=curve_exe_js_all[:,6:]

	
		curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,relative_path_exe,relative_path_exe_R=form_relative_path(curve_exe_js1,curve_exe_js2,robot1,robot2)
		lam=calc_lam_cs(relative_path_exe)

		###speed filter
		speed=np.gradient(lam)/np.gradient(timestamp)
		speed=moving_average(speed,padding=True)


		return lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R


	def chop_extension(self,curve_exe, curve_exe_R,curve_exe_js, speed, timestamp,p_start,p_end):
		start_idx=np.argmin(np.linalg.norm(p_start-curve_exe,axis=1))
		end_idx=np.argmin(np.linalg.norm(p_end-curve_exe,axis=1))

		#make sure extension doesn't introduce error
		if np.linalg.norm(curve_exe[start_idx]-p_start)>0.5:
			start_idx+=1
		if np.linalg.norm(curve_exe[end_idx]-p_end)>0.5:
			end_idx-=1

		curve_exe=curve_exe[start_idx:end_idx+1]
		curve_exe_js=curve_exe_js[start_idx:end_idx+1]
		curve_exe_R=curve_exe_R[start_idx:end_idx+1]
		speed=speed[start_idx:end_idx+1]
		lam=calc_lam_cs(curve_exe)

		return lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp[start_idx:end_idx+1]-timestamp[start_idx]

	def chop_extension_dual(self,lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R,p_start,p_end):
		start_idx=np.argmin(np.linalg.norm(p_start-relative_path_exe,axis=1))
		end_idx=np.argmin(np.linalg.norm(p_end-relative_path_exe,axis=1))

		#make sure extension doesn't introduce error
		if np.linalg.norm(relative_path_exe[start_idx]-p_start)>0.5:
			start_idx+=1
		if np.linalg.norm(relative_path_exe[end_idx]-p_end)>0.5:
			end_idx-=1

		curve_exe1=curve_exe1[start_idx:end_idx+1]
		curve_exe2=curve_exe2[start_idx:end_idx+1]
		curve_exe_R1=curve_exe_R1[start_idx:end_idx+1]
		curve_exe_R2=curve_exe_R2[start_idx:end_idx+1]
		curve_exe_js1=curve_exe_js1[start_idx:end_idx+1]
		curve_exe_js2=curve_exe_js2[start_idx:end_idx+1]

		relative_path_exe=relative_path_exe[start_idx:end_idx+1]
		relative_path_exe_R=relative_path_exe_R[start_idx:end_idx+1]

		timestamp=timestamp[start_idx:end_idx+1]
		speed=speed[start_idx:end_idx+1]
		lam=calc_lam_cs(relative_path_exe)



		return lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R

	def calc_robot2_q_from_blade_pose(self,blade_pose,base2_R,base2_p):
		R2=base2_R.T@blade_pose[:3,:3]
		p2=-base2_R.T@(base2_p-blade_pose[:3,-1])

		return self.robot2.inv(p2,R2)[0]


def main():
	return
if __name__ == "__main__":
	main()
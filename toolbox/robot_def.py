from general_robotics_toolbox import * 
from general_robotics_toolbox import tesseract as rox_tesseract
from general_robotics_toolbox import robotraconteur as rr_rox

import numpy as np
import yaml, copy,time
import pickle

def Rx(theta):
	return np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
def Ry(theta):
	return np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
def Rz(theta):
	return np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])

def Rp2H(p,R):
	###return homogeneous transformation matrix from R and p
	return np.vstack((np.hstack((R,np.array([p]).T)),np.array([0,0,0,1])))

class Transform_all(object):
	def __init__(self, p_all, R_all):
		self.R_all=np.array(R_all)
		self.p_all=np.array(p_all)

class robot_obj(object):
	###robot object class
	def __init__(self,def_path,tool_file_path='',base_transformation_file='',d=0,acc_dict_path=''):
		#def_path: robot 			definition yaml file, name must include robot vendor
		#tool_file_path: 			tool transformation to robot flange csv file
		#base_transformation_file: 	base transformation to world frame csv file
		#d: 						tool z extension
		#acc_dict_path: 			accleration profile

		with open(def_path, 'r') as f:
			robot = rr_rox.load_robot_info_yaml_to_robot(f)

		if len(tool_file_path)>0:
			tool_H=np.loadtxt(tool_file_path,delimiter=',')
			robot.R_tool=tool_H[:3,:3]
			robot.p_tool=tool_H[:3,-1]+np.dot(tool_H[:3,:3],np.array([0,0,d]))		

		if len(base_transformation_file)>0:
			self.base_H=np.loadtxt(base_transformation_file,delimiter=',')
		else:
			self.base_H=np.eye(4)

		if len(robot.joint_names)>6:	#redundant kinematic chain
			tesseract_robot = rox_tesseract.TesseractRobot(robot, "robot", invkin_solver="KDL")
		elif 'UR' in def_path:			#UR
			tesseract_robot = rox_tesseract.TesseractRobot(robot, "robot", invkin_solver="URInvKin")
		else:							#sepherical joint
			tesseract_robot = rox_tesseract.TesseractRobot(robot, "robot", invkin_solver="OPWInvKin")

		self.tesseract_robot=tesseract_robot


		###set attributes
		self.upper_limit=robot.joint_upper_limit 
		self.lower_limit=robot.joint_lower_limit 
		self.joint_vel_limit=robot.joint_vel_limit 
		self.joint_acc_limit=robot.joint_acc_limit 

		###acceleration table
		if len(acc_dict_path)>0:
			acc_dict= pickle.load(open(acc_dict_path,'rb'))
			q2_config=[]
			q3_config=[]
			q1_acc_n=[]
			q1_acc_p=[]
			q2_acc_n=[]
			q2_acc_p=[]
			q3_acc_n=[]
			q3_acc_p=[]
			for key, value in acc_dict.items():
			   q2_config.append(key[0])
			   q3_config.append(key[1])
			   q1_acc_n.append(value[0])
			   q1_acc_p.append(value[1])
			   q2_acc_n.append(value[2])
			   q2_acc_p.append(value[3])
			   q3_acc_n.append(value[4])
			   q3_acc_p.append(value[5])
			self.q2q3_config=np.array([q2_config,q3_config]).T
			self.q1q2q3_acc=np.array([q1_acc_n,q1_acc_p,q2_acc_n,q2_acc_p,q3_acc_n,q3_acc_p]).T

	def get_acc(self,q_all,direction=[]):
		###get acceleration limit from q config, assume last 3 joints acc fixed direction is 3 length vector, 0 is -, 1 is +
		#if a single point
		if q_all.ndim==1:
			###find closest q2q3 config, along with constant last 3 joints acc
			idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[1:3],axis=1))
			acc_lim=[]
			for d in direction:
				acc_lim.append(self.q1q2q3_acc[idx][2*len(acc_lim)+d])

			return np.append(acc_lim,self.joint_acc_limit[-3:])
		#if a list of points
		else:
			dq=np.gradient(q_all,axis=0)[:,:3]
			direction=(np.sign(dq)+1)/2
			direction=direction.astype(int)
			acc_limit_all=[]
			for i in range(len(q_all)):
				idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[i][1:3],axis=1))
				acc_lim=[]
				for d in direction[i]:
					acc_lim.append(self.q1q2q3_acc[idx][2*len(acc_lim)+d])

				acc_limit_all.append(np.append(acc_lim,self.joint_acc_limit[-3:]))

		return np.array(acc_limit_all)

	def fwd(self,q_all,world=False):
		###robot forworld kinematics
		#q_all:			robot joint angles or list of robot joint angles
		#world:			bool, if want to get coordinate in world frame or robot base frame
		if q_all.ndim==1:
			q=q_all
			pose_temp=self.tesseract_robot.fwdkin(q)	

			if world:
				pose_temp.p=self.base_H[:3,:3]@pose_temp.p+self.base_H[:3,-1]
				pose_temp.R=self.base_H[:3,:3]@pose_temp.R
			return pose_temp
		else:
			pose_p_all=[]
			pose_R_all=[]
			for q in q_all:
				pose_temp=self.tesseract_robot.fwdkin(q)	
				if world:
					pose_temp.p=self.base_H[:3,:3]@pose_temp.p+self.base_H[:3,-1]
					pose_temp.R=self.base_H[:3,:3]@pose_temp.R

				pose_p_all.append(pose_temp.p)
				pose_R_all.append(pose_temp.R)

			return Transform_all(pose_p_all,pose_R_all)
	
	def jacobian(self,q):
		return self.tesseract_robot.jacobian(q)

	def inv(self,p,R,last_joints=[]):
		if len(last_joints)==0:
			return self.tesseract_robot.invkin(Transform(R,p),np.zeros(len(self.joint_vel_limit)))
		else:	###sort solutions
			theta_v=self.tesseract_robot.invkin(Transform(R,p),last_joints)
			theta_dist = np.linalg.norm(np.subtract(theta_v,last_joints), axis=1)

        	return [theta_v[i] for i in list(np.argsort(theta_dist))]

def main():
	robot=robot_obj('../config/abb_6640_180_255_robot_default_config.yml',tool_file_path='../config/paintgun.csv',d=50,acc_dict_path='')
	p=np.array([1445.00688987, -248.17799722, 1037.37341832])
	R=np.array([[-0.83395293, -0.1490643,  -0.53132131],
				[ 0.17227772,  0.84437554, -0.50729709],
				[ 0.52425461, -0.51459672, -0.678489  ]])
	q = np.array([0.1, 0.11, 0.12, 0.13, 0.14, 0.15])
	now=time.time()
	for i in range(100):
		# print(robot.fwd(q))
		robot.fwd(q)
	print(time.time()-now)
	now=time.time()
	for i in range(100):
		# print(robot.inv(p,R,last_joints=np.array([ 0.0859182,   0.09685281,  0.28419715,  2.56388261, -1.34470404, -3.0320356 ])))
		robot.inv(p,R,last_joints=np.array([ 0.0859182,   0.09685281,  0.28419715,  2.56388261, -1.34470404, -3.0320356 ]))
	print(time.time()-now)
	return

if __name__ == '__main__':
	main()
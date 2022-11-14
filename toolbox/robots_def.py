from general_robotics_toolbox import * 
from general_robotics_toolbox import tesseract as rox_tesseract
from general_robotics_toolbox import robotraconteur as rr_rox

import numpy as np
import yaml, copy, time
import pickle

def Rx(theta):
	return np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
def Ry(theta):
	return np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
def Rz(theta):
	return np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
ex=np.array([[1.],[0.],[0.]])
ey=np.array([[0.],[1.],[0.]])
ez=np.array([[0.],[0.],[1.]])

def unwrapped_angle_check(q_init,q_all):

    temp_q=q_all-q_init
    temp_q = np.unwrap(temp_q)
    order=np.argsort(np.linalg.norm(temp_q,axis=1))
    # return q_all[order[0]]
    return temp_q[order[0]]+q_init

class robot_obj(object):
	###robot object class
	def __init__(self,robot_name,def_path,tool_file_path='',base_transformation_file='',d=0,acc_dict_path='',j_compensation=[1,1,1,1,1,1]):
		#def_path: robot 			definition yaml file, name must include robot vendor
		#tool_file_path: 			tool transformation to robot flange csv file
		#base_transformation_file: 	base transformation to world frame csv file
		#d: 						tool z extension
		#acc_dict_path: 			accleration profile

		self.robot_name=robot_name
		with open(def_path, 'r') as f:
			self.robot = rr_rox.load_robot_info_yaml_to_robot(f)

		self.def_path=def_path
		#define robot without tool
		self.robot_def_nT=Robot(self.robot.H,self.robot.P,self.robot.joint_type)

		if len(tool_file_path)>0:
			tool_H=np.loadtxt(tool_file_path,delimiter=',')
			self.robot.R_tool=tool_H[:3,:3]
			self.robot.p_tool=tool_H[:3,-1]+np.dot(tool_H[:3,:3],np.array([0,0,d]))
			self.p_tool=self.robot.p_tool
			self.R_tool=self.robot.R_tool

		if len(base_transformation_file)>0:
			self.base_H=np.loadtxt(base_transformation_file,delimiter=',')
		else:
			self.base_H=np.eye(4)

		###set attributes
		self.upper_limit=self.robot.joint_upper_limit 
		self.lower_limit=self.robot.joint_lower_limit 
		self.joint_vel_limit=self.robot.joint_vel_limit 
		self.joint_acc_limit=self.robot.joint_acc_limit 

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
				q1_acc_n.append(value[0%len(value)])
				q1_acc_p.append(value[1%len(value)])
				q2_acc_n.append(value[2%len(value)])
				q2_acc_p.append(value[3%len(value)])
				q3_acc_n.append(value[4%len(value)])
				q3_acc_p.append(value[5%len(value)])
			self.q2q3_config=np.array([q2_config,q3_config]).T
			self.q1q2q3_acc=np.array([q1_acc_n,q1_acc_p,q2_acc_n,q2_acc_p,q3_acc_n,q3_acc_p]).T
		
		###initialize tesseract robot
		self.initialize_tesseract_robot()

		## joint H compensation
		self.j_compensation=np.array(j_compensation)

	def initialize_tesseract_robot(self):
		if len(self.robot.joint_names)>6:	#redundant kinematic chain
			tesseract_robot = rox_tesseract.TesseractRobot(self.robot, "robot", invkin_solver="KDL")
		elif 'UR' in self.def_path:			#UR
			tesseract_robot = rox_tesseract.TesseractRobot(self.robot, "robot", invkin_solver="URInvKin")
		else:							#sepherical joint
			tesseract_robot = rox_tesseract.TesseractRobot(self.robot, "robot", invkin_solver="OPWInvKin")
		self.tesseract_robot=tesseract_robot

	def __getstate__(self):
		state = self.__dict__.copy()
		del state['tesseract_robot']
		return state

	def __setstate__(self, state):
		# Restore instance attributes (tesseract).
		self.__dict__.update(state)
		self.initialize_tesseract_robot()


	# def get_acc_old(self,q_all,direction=[]):
	# 	#if a single point
	# 	if q_all.ndim==1:
	# 		###find closest q2q3 config, along with constant last 3 joints acc
	# 		idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[1:3],axis=1))
	# 		return np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:])
	# 	else:
	# 		acc_limit_all=[]
	# 		for q in q_all:
	# 			idx=np.argmin(np.linalg.norm(self.q2q3_config-q[1:3],axis=1))
	# 			acc_limit_all.append(np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:]))

	# 	return np.array(acc_limit_all)

	def get_acc(self,q_all,direction=[]):
		###get acceleration limit from q config, assume last 3 joints acc fixed direction is 3 length vector, 0 is -, 1 is +
		#if a single point
		if q_all.ndim==1:
			###find closest q2q3 config, along with constant last 3 joints acc
			idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[1:3],axis=1))
			acc_lim=[]
			if len(direction)==0:
				raise AssertionError('direciton not provided')
				return
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

	def fwd(self,q_all,world=False,qlim_override=False):
		###robot forworld kinematics
		#q_all:			robot joint angles or list of robot joint angles
		#world:			bool, if want to get coordinate in world frame or robot base frame
		q_all=np.array(q_all)
		if q_all.ndim==1:
			q=q_all
			pose_temp=self.tesseract_robot.fwdkin(np.multiply(q,self.j_compensation))	

			if world:
				pose_temp.p=self.base_H[:3,:3]@pose_temp.p+self.base_H[:3,-1]
				pose_temp.R=self.base_H[:3,:3]@pose_temp.R
			return pose_temp
		else:
			pose_p_all=[]
			pose_R_all=[]
			for q in q_all:
				pose_temp=self.tesseract_robot.fwdkin(np.multiply(q,self.j_compensation))	
				if world:
					pose_temp.p=self.base_H[:3,:3]@pose_temp.p+self.base_H[:3,-1]
					pose_temp.R=self.base_H[:3,:3]@pose_temp.R

				pose_p_all.append(pose_temp.p)
				pose_R_all.append(pose_temp.R)

			return Transform_all(pose_p_all,pose_R_all)
	
	def fwd_j456(self,q):
		q = np.multiply(q,self.j_compensation)
		if (self.robot.joint_lower_limit is not None and self.robot.joint_upper_limit is not None):
			assert np.greater_equal(q, self.robot.joint_lower_limit).all(), "Specified joints out of range"
			assert np.less_equal(q, self.robot.joint_upper_limit).all(), "Specified joints out of range"

		p = self.robot.P[:,[1]]
		R = np.identity(3)
		for i in xrange(1,len(self.robot.joint_type)-1):
			R = R.dot(rot(self.robot.H[:,[i]],q[i]))
			p = p + R.dot(self.robot.P[:,[i+1]])
		p=np.reshape(p,(3,))

		return Transform(R, p)

	def jacobian(self,q):
		return self.tesseract_robot.jacobian(np.multiply(q,self.j_compensation))

	def inv(self,p,R,last_joints=[]):
		# self.check_tesseract_robot()
		if len(last_joints)==0:
			return np.multiply(self.tesseract_robot.invkin(Transform(R,p),np.zeros(len(self.joint_vel_limit))),self.j_compensation)
		else:	###sort solutions
			last_joints=np.multiply(last_joints,self.j_compensation)
			theta_v=self.tesseract_robot.invkin(Transform(R,p),last_joints)
			eq_theta_v=equivalent_configurations(self.robot, theta_v, last_joints)
			theta_v.extend(eq_theta_v)
			theta_dist = np.linalg.norm(np.subtract(theta_v,last_joints), axis=1)
			return [np.multiply(theta_v[i],self.j_compensation) for i in list(np.argsort(theta_dist))]

			
#ALL in mm
class abb6640(object):
	#default tool paintgun
	def __init__(self,R_tool=Ry(np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0,acc_dict_path=''):
		###ABB IRB 6640 180/2.55 Robot Definition
		self.H=np.concatenate((ez,ey,ey,ex,ey,ex),axis=1)
		p0=np.array([[0],[0],[0.78]])
		p1=np.array([[0.32],[0],[0]])
		p2=np.array([[0.],[0],[1.075]])
		p3=np.array([[0],[0],[0.2]])   
		p4=np.array([[1.1425],[0],[0]])
		p5=np.array([[0.2],[0],[0]])
		p6=np.array([[0.0],[0],[0.0]])

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))
		
		self.R_tool=R_tool
		self.p_tool=tcp_new


		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		
		###updated range&vel limit
		self.upper_limit=np.radians([170.,85.,70.,300.,120.,360.])
		self.lower_limit=np.radians([-170.,-65.,-180.,-300.,-120.,-360.])
		self.joint_vel_limit=np.radians([100,90,90,190,140,190])
		self.joint_acc_limit=np.array([-1,-1,-1,42.49102688076435,36.84030926197994,50.45298947544431])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit,joint_acc_limit=self.joint_acc_limit, R_tool=R_tool,p_tool=tcp_new)
		# self.robot_def_tess=tesseract.TesseractRobot(self.robot_def,invkin_solver = "OPWInvKin")
		self.robot_def_nT=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit)

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


	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0]),qlim_override=False):
		if qlim_override:
			robot_def=copy.deepcopy(self.robot_def)
			robot_def.joint_upper_limit=999*np.ones(len(self.upper_limit))
			robot_def.joint_lower_limit=-999*np.ones(len(self.lower_limit))
			pose_temp=fwdkin(robot_def,q)
			# pose_temp=self.robot_def_tess.fwdkin(q)
		else:
			pose_temp=fwdkin(self.robot_def,q)
			# pose_temp=self.robot_def_tess.fwdkin(q)

		pose_temp.p=base_R@pose_temp.p+base_p
		pose_temp.R=base_R@pose_temp.R
		return pose_temp

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=self.fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		
		# q_all=self.robot_def.tess.invkin(pose,last_joints)
		return q_all


class abb1200(object):
	#default tool paintgun
	def __init__(self,R_tool=Ry(np.radians(90)),p_tool=np.zeros(3),d=0,acc_dict_path=''):
		###ABB IRB 1200 5/0.9 Robot Definition
		self.H=np.concatenate((ez,ey,ey,ex,ey,ex),axis=1)
		p0=np.array([[0],[0],[0.3991]])
		p1=np.array([[0],[0],[0]])
		p2=np.array([[0.],[0],[0.448]])
		p3=np.array([[0],[0],[0.042]])
		p4=np.array([[0.451],[0],[0]])
		p5=np.array([[0.082],[0],[0]])
		p6=np.array([[0],[0],[0]])

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))

		self.R_tool=R_tool
		self.p_tool=tcp_new

		###updated range&vel limit
		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		self.upper_limit=np.radians([170.,130.,70.,270.,130.,360.])
		self.lower_limit=np.radians([-170.,-100.,-200.,-270.,-130.,-360.])
		self.joint_vel_limit=np.radians([288,240,297,400,405,600])
		self.joint_acc_limit=np.array([-1,-1,-1,85.73244187330907,126.59979534862278,167.56543454239707])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)
		self.robot_def_nT=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit)

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

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0]),qlim_override=False):
		if qlim_override:
			robot_def=copy.deepcopy(self.robot_def)
			robot_def.joint_upper_limit=999*np.ones(len(self.upper_limit))
			robot_def.joint_lower_limit=-999*np.ones(len(self.lower_limit))
			pose_temp=fwdkin(robot_def,q)
		else:
			pose_temp=fwdkin(self.robot_def,q)

		
		pose_temp.p=base_R@pose_temp.p+base_p
		pose_temp.R=base_R@pose_temp.R
		return pose_temp

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=self.fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		return q_all

class m900ia(object):
	#default tool paintgun
	def __init__(self,R_tool=Ry(np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0):
		###FANUC m900iA 350 Robot Definition
		self.H=np.concatenate((ez,ey,-ey,-ex,-ey,-ex),axis=1)
		p0=np.array([[0],[0],[0.95]])
		p1=np.array([[0.37],[0],[0]])
		p2=np.array([[0.],[0],[1.050]])
		p3=np.array([[0],[0],[0.2]])   
		p4=np.array([[1.250],[0],[0]])
		p5=np.array([[0.27],[0],[0]])
		p6=np.array([[0.0],[0],[0.0]])

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))

		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		
		###updated range&vel limit
		self.upper_limit=np.radians([180.,75.,65.,360.,125.,360.])
		self.lower_limit=np.radians([-180.,-75.,-58.,-360.,-125.,-360.])
		self.joint_vel_limit=np.radians([100.,95.,95.,105.,105.,170.])
		# self.joint_acc_limit=np.radians([300.,561.,743.,244.,319.,243.])
		self.joint_acc_limit=np.radians([183.333,175,166.666,183.333,183.333,300])
		# self.joint_jrk_limit=np.radians([1020.408,765.306,765.306,1434.949,1434.949,1434.949])
		self.joint_jrk_limit=np.radians([611.111,583.333,555.555,611.111,611.111,1000])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_temp=fwdkin(self.robot_def,q)
		pose_temp.p=np.dot(base_R,pose_temp.p)+base_p
		pose_temp.R=np.dot(base_R,pose_temp.R)
		return pose_temp
	
	def fwd_j456(self,q):
		if (self.robot_def.joint_lower_limit is not None and self.robot_def.joint_upper_limit is not None):
			assert np.greater_equal(q, self.robot_def.joint_lower_limit).all(), "Specified joints out of range"
			assert np.less_equal(q, self.robot_def.joint_upper_limit).all(), "Specified joints out of range"

		p = self.robot_def.P[:,[1]]
		R = np.identity(3)
		for i in xrange(1,len(self.robot_def.joint_type)-1):
			R = R.dot(rot(self.robot_def.H[:,[i]],q[i]))
			p = p + R.dot(self.robot_def.P[:,[i+1]])
		p=np.reshape(p,(3,))

		return Transform(R, p)

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		return q_all

class m710ic(object):
	#default tool paintgun
	def __init__(self,R_tool=Ry(np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0):
		###FANUC m710ic 70 Robot Definition
		self.H=np.concatenate((ez,ey,-ey,-ex,-ey,-ex),axis=1)
		p0=np.array([[0],[0],[0.565]])
		p1=np.array([[0.15],[0],[0]])
		p2=np.array([[0.],[0],[0.870]])
		p3=np.array([[0],[0],[0.17]])   
		p4=np.array([[1.016],[0],[0]])
		p5=np.array([[0.175],[0],[0]])
		p6=np.array([[0.0],[0],[0.0]])

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))

		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		
		###updated range&vel limit
		self.upper_limit=np.radians([180.,135,205.,360.,125.,360.])
		self.lower_limit=np.radians([-180.,-90.,-80.,-360.,-125.,-360.])
		self.joint_vel_limit=np.radians([160.,120.,120.,225.,225.,225.])
		# self.joint_acc_limit=np.radians([640.,520.,700.,910.,910.,1207.])
		self.joint_acc_limit=np.radians([285.741,214.286,214.286,401.786,401.786,401.786])
		self.joint_jrk_limit=np.radians([1020.408,765.306,765.306,1434.949,1434.949,1434.949])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_temp=fwdkin(self.robot_def,q)
		pose_temp.p=np.dot(base_R,pose_temp.p)+base_p
		pose_temp.R=np.dot(base_R,pose_temp.R)
		return pose_temp
	
	def fwd_j456(self,q):
		if (self.robot_def.joint_lower_limit is not None and self.robot_def.joint_upper_limit is not None):
			assert np.greater_equal(q, self.robot_def.joint_lower_limit).all(), "Specified joints out of range"
			assert np.less_equal(q, self.robot_def.joint_upper_limit).all(), "Specified joints out of range"

		p = self.robot_def.P[:,[1]]
		R = np.identity(3)
		for i in xrange(1,len(self.robot_def.joint_type)-1):
			R = R.dot(rot(self.robot_def.H[:,[i]],q[i]))
			p = p + R.dot(self.robot_def.P[:,[i+1]])
		p=np.reshape(p,(3,))

		return Transform(R, p)

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		return q_all

class m10ia(object):
	#default tool paintgun
	def __init__(self,R_tool=Ry(np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0,acc_dict_path=''):
		###FANUC m710ic 70 Robot Definition
		self.H=np.concatenate((ez,ey,-ey,-ex,-ey,-ex),axis=1)
		p0=np.array([[0],[0],[0.45]])
		# p0=np.array([[0],[0],[0]])
		p1=np.array([[0.15],[0],[0]])
		p2=np.array([[0.],[0],[0.6]])
		p3=np.array([[0],[0],[0.2]])   
		p4=np.array([[0.64],[0],[0]])
		p5=np.array([[0.1],[0],[0]])
		p6=np.array([[0.0],[0],[0.0]])

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))

		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		
		###updated range&vel limit
		self.upper_limit=np.radians([170.,160,180.,190.,140.,360.])
		self.lower_limit=np.radians([-170.,-90.,-89.,-190.,-140.,-360.])
		self.joint_vel_limit=np.radians([210.,190.,210.,400.,400.,600.])
		# self.joint_acc_limit=np.radians([640.,520.,700.,910.,910.,1207.])
		self.joint_acc_limit=np.radians([285.741,214.286,214.286,401.786,401.786,401.786])
		self.joint_jrk_limit=np.radians([1020.408,765.306,765.306,1434.949,1434.949,1434.949])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

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

				if len(value) > 3:
					q1_acc_n.append(value[0])
					q1_acc_p.append(value[1])
					q2_acc_n.append(value[2])
					q2_acc_p.append(value[3])
					q3_acc_n.append(value[4])
					q3_acc_p.append(value[5])
				else:
					q1_acc_n.append(value[0])
					q1_acc_p.append(value[0])
					q2_acc_n.append(value[1])
					q2_acc_p.append(value[1])
					q3_acc_n.append(value[2])
					q3_acc_p.append(value[2])
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

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_temp=fwdkin(self.robot_def,q)
		pose_temp.p=np.dot(base_R,pose_temp.p)+base_p
		pose_temp.R=np.dot(base_R,pose_temp.R)
		return pose_temp
	
	def fwd_j456(self,q):
		if (self.robot_def.joint_lower_limit is not None and self.robot_def.joint_upper_limit is not None):
			assert np.greater_equal(q, self.robot_def.joint_lower_limit).all(), "Specified joints out of range"
			assert np.less_equal(q, self.robot_def.joint_upper_limit).all(), "Specified joints out of range"

		p = self.robot_def.P[:,[1]]
		R = np.identity(3)
		for i in xrange(1,len(self.robot_def.joint_type)-1):
			R = R.dot(rot(self.robot_def.H[:,[i]],q[i]))
			p = p + R.dot(self.robot_def.P[:,[i+1]])
		p=np.reshape(p,(3,))

		return Transform(R, p)

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		return q_all

class lrmate200id(object):
	#default tool paintgun
	def __init__(self,R_tool=Ry(np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0,acc_dict_path=''):
		###FANUC m710ic 70 Robot Definition
		self.H=np.concatenate((ez,ey,-ey,-ex,-ey,-ex),axis=1)
		p0=np.array([[0],[0],[0.33]])
		# p0=np.array([[0],[0],[0]])
		p1=np.array([[0.05],[0],[0]])
		p2=np.array([[0.],[0],[0.33]])
		p3=np.array([[0],[0],[0.035]])   
		p4=np.array([[0.335],[0],[0]])
		p5=np.array([[0.08],[0],[0]])
		p6=np.array([[0.0],[0],[0.0]])

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))

		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		
		###updated range&vel limit
		self.upper_limit=np.radians([170.,145,205.,190.,125.,360.])
		self.lower_limit=np.radians([-170.,-100.,-70.,-190.,-125.,-360.])
		self.joint_vel_limit=np.radians([450.,380.,520.,550.,545.,1000.])
		# self.joint_acc_limit=np.radians([640.,520.,700.,910.,910.,1207.])
		self.joint_acc_limit=np.radians([285.741,214.286,214.286,401.786,401.786,401.786])
		self.joint_jrk_limit=np.radians([1020.408,765.306,765.306,1434.949,1434.949,1434.949])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

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

				if len(value) > 3:
					q1_acc_n.append(value[0])
					q1_acc_p.append(value[1])
					q2_acc_n.append(value[2])
					q2_acc_p.append(value[3])
					q3_acc_n.append(value[4])
					q3_acc_p.append(value[5])
				else:
					q1_acc_n.append(value[0])
					q1_acc_p.append(value[0])
					q2_acc_n.append(value[1])
					q2_acc_p.append(value[1])
					q3_acc_n.append(value[2])
					q3_acc_p.append(value[2])
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

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_temp=fwdkin(self.robot_def,q)
		pose_temp.p=np.dot(base_R,pose_temp.p)+base_p
		pose_temp.R=np.dot(base_R,pose_temp.R)
		return pose_temp
	
	def fwd_j456(self,q):
		if (self.robot_def.joint_lower_limit is not None and self.robot_def.joint_upper_limit is not None):
			assert np.greater_equal(q, self.robot_def.joint_lower_limit).all(), "Specified joints out of range"
			assert np.less_equal(q, self.robot_def.joint_upper_limit).all(), "Specified joints out of range"

		p = self.robot_def.P[:,[1]]
		R = np.identity(3)
		for i in xrange(1,len(self.robot_def.joint_type)-1):
			R = R.dot(rot(self.robot_def.H[:,[i]],q[i]))
			p = p + R.dot(self.robot_def.P[:,[i+1]])
		p=np.reshape(p,(3,))

		return Transform(R, p)

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		return q_all

class arb_robot(object):
	#R_tool make tool z pointing to +x at 0 config
	def __init__(self, H,P,joint_type,upper_limit,lower_limit, joint_vel_limit,R_tool=Ry(np.radians(90)),p_tool=np.zeros(3),d=0,acc_dict_path=''):
		###All in mm
		self.H=H
		self.P=P

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))

		self.R_tool=R_tool
		self.p_tool=tcp_new

		###updated range&vel limit
		self.joint_type=joint_type
		self.upper_limit=upper_limit
		self.lower_limit=lower_limit
		self.joint_vel_limit=joint_vel_limit
		self.joint_acc_limit=10*self.joint_vel_limit
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

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
		
	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0]),qlim_override=False):
		if qlim_override:
			robot_def=copy.deepcopy(self.robot_def)
			robot_def.joint_upper_limit=999*np.ones(len(self.upper_limit))
			robot_def.joint_lower_limit=-999*np.ones(len(self.lower_limit))
			pose_temp=fwdkin(robot_def,q)
		else:
			pose_temp=fwdkin(self.robot_def,q)
		pose_temp=fwdkin(self.robot_def,q)
		pose_temp.p=np.dot(base_R,pose_temp.p)+base_p
		pose_temp.R=np.dot(base_R,pose_temp.R)
		return pose_temp

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		return q_all

def yml2robdef(file):
	robot_yml=yaml.full_load(file)
	kin_chain=robot_yml['chains'][0]
	joint_info=robot_yml['joint_info']
	tool_pose=kin_chain['flange_pose']

	###kin chain
	H = []
	P = []

	for i in range(len(kin_chain['H'])):
		H.append(list(kin_chain['H'][i].values()))
		P.append(list(kin_chain['P'][i].values()))
	P.append(list(kin_chain['P'][-1].values()))
	H=np.array(H).reshape((len(kin_chain['H']),3)).T
	P=np.array(P).reshape((len(kin_chain['P']),3)).T*1000	###make sure in mm

	###joint info
	joint_type=[]	
	upper_limit=[]
	lower_limit=[]
	joint_vel_limit=[]
	for i in range(len(joint_info)):
		joint_type.append(0 if joint_info[i]['joint_type']=='revolute' else 1)
		upper_limit.append(joint_info[i]['joint_limits']['upper'])
		lower_limit.append(joint_info[i]['joint_limits']['lower'])
		joint_vel_limit.append(joint_info[i]['joint_limits']['velocity'])

	###tool pose
	R_tool=q2R(list(tool_pose['orientation'].values()))
	p_tool=np.array(list(tool_pose['position'].values()))*1000

	###create a robot
	robot=arb_robot(H,P,joint_type,upper_limit,lower_limit, joint_vel_limit,R_tool=R_tool,p_tool=p_tool)

	return robot
class Transform_all(object):
	def __init__(self, p_all, R_all):
		self.R_all=np.array(R_all)
		self.p_all=np.array(p_all)



def HomogTrans(q,h,p,jt):

	if jt==0:
		H=np.vstack((np.hstack((rot(h,q), p.reshape((3,1)))),np.array([0, 0, 0, 1,])))
	else:
		H=np.vstack((np.hstack((np.eye(3), p + np.dot(q, h))),np.array([0, 0, 0, 1,])))
	return H
def Hvec(h,jtype):

	if jtype>0:
		H=np.vstack((np.zeros((3,1)),h))
	else:
		H=np.vstack((h.reshape((3,1)),np.zeros((3,1))))
	return H
def phi(R,p):

	Phi=np.vstack((np.hstack((R,np.zeros((3,3)))),np.hstack((-np.dot(R,hat(p)),R))))
	return Phi


def jdot(q,qdot):
	zv=np.zeros((3,1))
	H=np.eye(4)
	J=[]
	Jdot=[]
	n=6
	Jmat=[]
	Jdotmat=[]
	for i in range(n+1):
		if i<n:
			hi=self.robot_def.H[:,i]
			qi=q[i]
			qdi=qdot[i]
			ji=self.robot_def.joint_type[i]

		else:
			qi=0
			qdi=0
			di=0
			ji=0

		Pi=self.robot_def.P[:,i]
		Hi=HomogTrans(qi,hi,Pi,ji)
		Hn=np.dot(H,Hi)
		H=Hn

		PHI=phi(Hi[:3,:3].T,Hi[:3,-1])
		Hveci=Hvec(hi,ji)
		###Partial Jacobian progagation
		if(len(J)>0):
			Jn=np.hstack((np.dot(PHI,J), Hveci))
			temp=np.vstack((np.hstack((hat(hi), np.zeros((3,3)))),np.hstack((np.zeros((3,3)),hat(hi)))))
			Jdotn=-np.dot(qdi,np.dot(temp,Jn)) + np.dot(PHI,np.hstack((Jdot, np.zeros(Hveci.shape))))
		else:
			Jn=Hveci
			Jdotn=np.zeros(Jn.shape)

		Jmat.append(Jn) 
		Jdotmat.append(Jdotn)
		J=Jn
		Jdot=Jdotn

	Jmat[-1]=Jmat[-1][:,:n]
	Jdotmat[-1]=Jdotmat[-1][:,:n]
	return Jdotmat[-1]

def main():
	robot=abb6640(d=50)
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
		# print(robot.inv(p,R,last_joints=[ 0.0859182,   0.09685281,  0.28419715,  2.56388261, -1.34470404, -3.0320356 ]))
		robot.inv(p,R,last_joints=[ 0.0859182,   0.09685281,  0.28419715,  2.56388261, -1.34470404, -3.0320356 ])
	print(time.time()-now)
	return

def invtest():
	# robot=abb6640(d=50)
	# last_joints=[-0.84190536,  0.61401203,  0.2305977,  -2.70622154, -0.74584949, -2.21577141]
	# pose=robot.fwd(last_joints)
	# print('correct: ',robot.inv(pose.p,pose.R,last_joints))
	# robot2=robot_obj('../config/abb_6640_180_255_robot_default_config.yml',tool_file_path='../config/paintgun.csv',d=50,acc_dict_path='')
	# theta_v=robot2.inv(pose.p,pose.R)
	# print('passed to tes:',theta_v[0])
	# print('equivalent_configurations: ',robot2.tesseract_robot.redundant_solutions(theta_v[0]))

	robot=abb6640(d=50)
	last_joints=[-0.84190536,  0.61401203,  0.2305977,  -2.70622154, -0.74584949, -2.21577141]
	pose=robot.fwd(last_joints)
	print('correct: ',robot.inv(pose.p,pose.R,last_joints))
	theta_v=robot.inv(pose.p,pose.R)
	print('inv solutions: ',theta_v)
	print('equivalent_configurations: ',equivalent_configurations(robot.robot_def, theta_v, last_joints))


if __name__ == '__main__':
	invtest()
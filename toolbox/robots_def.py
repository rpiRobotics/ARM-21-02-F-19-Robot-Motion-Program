from general_robotics_toolbox import *
import numpy as np
import yaml, copy
import pickle

def Rx(theta):
	return np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
def Ry(theta):
	return np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
def Rz(theta):
	return np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
ex=np.array([[1],[0],[0]])
ey=np.array([[0],[1],[0]])
ez=np.array([[0],[0],[1]])

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
		# self.joint_acc_limit=np.radians([312,292,418,2407,1547,3400])
		self.joint_acc_limit=np.array([-1,-1,-1,42.49102688076435,36.84030926197994,50.45298947544431])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

		###acceleration table
		if len(acc_dict_path)>0:
			acc_dict= pickle.load(open(acc_dict_path,'rb'))
			q2_config=[]
			q3_config=[]
			q1_acc=[]
			q2_acc=[]
			q3_acc=[]
			for key, value in acc_dict.items():
				q2_config.append(key[0])
				q3_config.append(key[1])
				q1_acc.append(value[0])
				q2_acc.append(value[1])
				q3_acc.append(value[2])
			self.q2q3_config=np.array([q2_config,q3_config]).T
			self.q1q2q3_acc=np.array([q1_acc,q2_acc,q3_acc]).T

	def get_acc(self,q_all):
		#if a single point
		if q_all.ndim==1:
			###find closest q2q3 config, along with constant last 3 joints acc
			idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[1:3],axis=1))
			return np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:])
		else:
			acc_limit_all=[]
			for q in q_all:
				idx=np.argmin(np.linalg.norm(self.q2q3_config-q[1:3],axis=1))
				acc_limit_all.append(np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:]))

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

		###acceleration table
		if len(acc_dict_path)>0:
			acc_dict= pickle.load(open(acc_dict_path,'rb'))
			q2_config=[]
			q3_config=[]
			q1_acc=[]
			q2_acc=[]
			q3_acc=[]
			for key, value in acc_dict.items():
				q2_config.append(key[0])
				q3_config.append(key[1])
				q1_acc.append(value[0])
				q2_acc.append(value[1])
				q3_acc.append(value[2])
			self.q2q3_config=np.array([q2_config,q3_config]).T
			self.q1q2q3_acc=np.array([q1_acc,q2_acc,q3_acc]).T

	def get_acc(self,q_all):
		#if a single point
		if q_all.ndim==1:
			###find closest q2q3 config, along with constant last 3 joints acc
			idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[1:3],axis=1))
			return np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:])
		else:
			acc_limit_all=[]
			for q in q_all:
				idx=np.argmin(np.linalg.norm(self.q2q3_config-q[1:3],axis=1))
				acc_limit_all.append(np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:]))

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
	def __init__(self,R_tool=Ry(np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0):
		###FANUC m710ic 70 Robot Definition
		self.H=np.concatenate((ez,ey,-ey,-ex,-ey,-ex),axis=1)
		# p0=np.array([[0],[0],[0.45]])
		p0=np.array([[0],[0],[0]])
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
		self.lower_limit=np.radians([-180.,-90.,-89.,-190.,-140.,-360.])
		self.joint_vel_limit=np.radians([210.,190.,210.,400.,400.,600.])
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
			q1_acc=[]
			q2_acc=[]
			q3_acc=[]
			for key, value in acc_dict.items():
				q2_config.append(key[0])
				q3_config.append(key[1])
				q1_acc.append(value[0])
				q2_acc.append(value[1])
				q3_acc.append(value[2])
			self.q2q3_config=np.array([q2_config,q3_config]).T
			self.q1q2q3_acc=np.array([q1_acc,q2_acc,q3_acc]).T

	def get_acc(self,q_all):
		#if a single point
		if q_all.ndim==1:
			###find closest q2q3 config, along with constant last 3 joints acc
			idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[1:3],axis=1))
			return np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:])
		else:
			acc_limit_all=[]
			for q in q_all:
				idx=np.argmin(np.linalg.norm(self.q2q3_config-q[1:3],axis=1))
				acc_limit_all.append(np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:]))

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
	print(robot.inv(p,R,last_joints=[ 0.0859182,   0.09685281,  0.28419715,  2.56388261, -1.34470404, -3.0320356 ]))
	return

if __name__ == '__main__':
	main()
from general_robotics_toolbox import *
import numpy as np
import yaml


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
	def __init__(self,R_tool=Ry(np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0):
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
		# print(tcp_new)


		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		
		###updated range&vel limit
		self.upper_limit=np.radians([170.,85.,70.,300.,120.,360.])
		self.lower_limit=np.radians([-170.,-65.,-180.,-300.,-120.,-360.])
		self.joint_vel_limit=np.radians([100,90,90,190,140,190])
		self.joint_acc_limit=np.radians([312,292,418,2407,1547,3400])
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0])):
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

	def inv(self,p,R=np.eye(3)):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose)
		return q_all


class abb1200(object):
	#default tool paintgun
	def __init__(self,R_tool=Ry(np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0):
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

		###updated range&vel limit
		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		self.upper_limit=np.radians([170.,130.,70.,270.,130.,360.])
		self.lower_limit=np.radians([-170.,-100.,-200.,-270.,-130.,-360.])
		self.joint_vel_limit=np.radians([288,240,297,400,405,600])
		self.joint_acc_limit=10*self.joint_vel_limit
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0])):
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

	def inv(self,p,R=np.eye(3)):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose)
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
		self.upper_limit=np.radians([180.,66.,66.,360.,125.,360.])
		self.lower_limit=np.radians([-180.,-65.,-66.,-360.,-125.,-360.])
		self.joint_vel_limit=np.radians([100.,95.,95.,105.,105.,170.])
		self.joint_acc_limit=np.radians([300.,561.,743.,244.,319.,243.])
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

	def inv(self,p,R=np.eye(3)):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose)
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
		self.lower_limit=np.radians([-180.,-80.,-80.,-360.,-125.,-360.])
		self.joint_vel_limit=np.radians([160.,120.,120.,225.,225.,225.])
		self.joint_acc_limit=np.radians([640.,520.,700.,910.,910.,1207.])
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

	def inv(self,p,R=np.eye(3)):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose)
		return q_all

class arb_robot(object):
	#R_tool make tool z pointing to +x at 0 config
	def __init__(self, H,P,joint_type,upper_limit,lower_limit, joint_vel_limit,R_tool=Ry(np.radians(90)),p_tool=np.zeros(3),d=0):
		###All in mm
		self.H=H
		self.P=P

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))

		###updated range&vel limit
		self.joint_type=joint_type
		self.upper_limit=upper_limit
		self.lower_limit=lower_limit
		self.joint_vel_limit=joint_vel_limit
		self.joint_acc_limit=10*self.joint_vel_limit
		self.robot_def=Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

	def jacobian(self,q):
		return robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0])):
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

	def inv(self,p,R=np.eye(3)):
		pose=Transform(R,p)
		q_all=robot6_sphericalwrist_invkin(self.robot_def,pose)
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
	abb6640(d=50)
	return

if __name__ == '__main__':
	main()
import numpy as np
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('../toolbox')
sys.path.append('toolbox')
from robots_def import *
from utils import *
from lambda_calc import *

def pose_opt(robot,curve,curve_normal):

	###get curve center and best place to center in robot frame
	C=np.average(curve,axis=0)
	p=robot.fwd(np.zeros(len(robot.joint_vel_limit))).p
	#arbitrary best center point for curve
	p[0]=p[0]/2
	p[-1]=p[-1]/2
	###determine Vy by eig(cov)
	curve_cov=np.cov(curve.T)
	eigenValues, eigenVectors = np.linalg.eig(curve_cov)
	idx = eigenValues.argsort()[::-1]   
	eigenValues = eigenValues[idx]
	eigenVectors = eigenVectors[:,idx]
	Vy=eigenVectors[0]
	###get average curve normal
	N=np.sum(curve_normal,axis=0)
	N=N/np.linalg.norm(N)
	###project N on to plane of Vy
	N=VectorPlaneProjection(N,Vy)

	###form transformation
	R=np.vstack((np.cross(Vy,-N),Vy,-N))
	T=p-R@C

	return H_from_RT(R,T)

def curve_frame_conversion(curve,curve_normal,H):
	curve_base=np.zeros(curve.shape)
	curve_normal_base=np.zeros(curve_normal.shape)

	for i in range(len(curve_base)):
		curve_base[i]=np.dot(H,np.hstack((curve[i],[1])).T)[:-1]

	#convert curve direction to base frame
	curve_normal_base=np.dot(H[:3,:3],curve_normal.T).T

	return curve_base,curve_normal_base

def find_js(robot,curve,curve_normal):
	###find all possible inv solution for given curve

	###get R first 
	curve_R=[]
	for i in range(len(curve)-1):
		R_curve=direction2R(curve_normal[i],-curve[i+1]+curve[i])	
		curve_R.append(R_curve)

	###insert initial orientation
	curve_R.insert(0,curve_R[0])

	###get all possible initial config
	try:
		q_inits=np.array(robot.inv(curve[0],curve_R[0]))
	except:
		traceback.print_exc()
		print('no solution available')
		return

	# print(np.degrees(q_inits))
	curve_js_all=[]
	for q_init in q_inits:
		curve_js=np.zeros((len(curve),6))
		curve_js[0]=q_init
		for i in range(1,len(curve)):
			q_all=np.array(robot.inv(curve[i],curve_R[i]))
			if len(q_all)==0:
				#if no solution
				print(i)
				print(np.degrees(curve_js[i-1]))
				print('no solution available')
				return

			temp_q=q_all-curve_js[i-1]
			order=np.argsort(np.linalg.norm(temp_q,axis=1))
			if np.linalg.norm(q_all[order[0]]-curve_js[i-1])>0.5:
				break	#if large changes in q
			else:
				curve_js[i]=q_all[order[0]]

		#check if all q found
		if np.linalg.norm(curve_js[-1])>0:
			curve_js_all.append(curve_js)

	return curve_js_all

if __name__ == "__main__":
	main()
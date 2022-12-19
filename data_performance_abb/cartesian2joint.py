import numpy as np
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('../toolbox')
from robots_def import *
from utils import *

def main():
	data_dir='wood/'

	col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z'] 
	data = read_csv(data_dir+"Curve_in_base_frame.csv", names=col_names)
	curve_x=data['X'].tolist()
	curve_y=data['Y'].tolist()
	curve_z=data['Z'].tolist()
	curve_direction_x=data['direction_x'].tolist()
	curve_direction_y=data['direction_y'].tolist()
	curve_direction_z=data['direction_z'].tolist()

	curve=np.vstack((curve_x, curve_y, curve_z)).T
	curve_direction=np.vstack((curve_direction_x, curve_direction_y, curve_direction_z)).T


	abb6640_obj=abb6640(d=50)

	curve_R=[]


	for i in range(len(curve)):
		try:
			R_curve=direction2R(curve_direction[i],-curve[i+1]+curve[i])
		except:
			traceback.print_exc()
			pass
		
		curve_R.append(R_curve)

	###insert initial orientation
	curve_R.insert(0,curve_R[0])
	

	try:
		q_inits=np.array(abb6640_obj.inv(curve[0],curve_R[0]))
	except:
		print('no solution available')
		return

	for q_init in q_inits:
		curve_js=np.zeros((len(curve),6))
		curve_js[0]=q_init
		for i in range(1,len(curve)):
			try:
				q_all=np.array(abb6640_obj.inv(curve[i],curve_R[i]))
			except:
				#if no solution
				print('no solution available')
				return

			temp_q=q_all-curve_js[i-1]
			order=np.argsort(np.linalg.norm(temp_q,axis=1))
			if np.linalg.norm(q_all[order[0]]-curve_js[i-1])>0.5:
				print('large change')
				break	#if large changes in q
			else:
				curve_js[i]=q_all[order[0]]

		#check if all q found
		if np.linalg.norm(curve_js[-1])>0:
			break


	###checkpoint3
	###make sure fwd(joint) and original curve match
	# H=np.vstack((np.hstack((R.T,-np.dot(R.T,T))),np.array([0,0,0,1])))
	# curve_temp=np.zeros(curve.shape)
	# for i in range(len(curve_js)):
	# 	curve_temp[i]=(np.dot(H,np.hstack((fwd(curve_js[i]).p,[1])).T)[:-1])
	# print(np.max(np.linalg.norm(curve-curve_temp,axis=1)))




	###output to csv
	df=DataFrame({'q0':curve_js[:,0],'q1':curve_js[:,1],'q2':curve_js[:,2],'q3':curve_js[:,3],'q4':curve_js[:,4],'q5':curve_js[:,5]})
	df.to_csv(data_dir+'Curve_js.csv',header=False,index=False)


def main2():
	sys.path.append('../constraint_solver')
	from constraint_solver import lambda_opt

	col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z'] 
	data = read_csv("from_ge/Curve_in_base_frame2.csv", names=col_names)
	curve_x=data['X'].tolist()
	curve_y=data['Y'].tolist()
	curve_z=data['Z'].tolist()
	curve_direction_x=data['direction_x'].tolist()
	curve_direction_y=data['direction_y'].tolist()
	curve_direction_z=data['direction_z'].tolist()

	curve=np.vstack((curve_x, curve_y, curve_z)).T
	curve_direction=np.vstack((curve_direction_x, curve_direction_y, curve_direction_z)).T

	opt=lambda_opt(curve,curve_direction,steps=len(curve),robot1=abb6640(d=50))
	R_temp=opt.direction2R(curve_direction[0],-curve[1]+curve[0])
	q_init=opt.robot1.inv(curve[0],R_temp)[0]
	q_out=opt.single_arm_stepwise_optimize(q_init)

	###output to csv
	df=DataFrame({'q0':q_out[:,0],'q1':q_out[:,1],'q2':q_out[:,2],'q3':q_out[:,3],'q4':q_out[:,4],'q5':q_out[:,5]})
	df.to_csv('from_ge/Curve_js2.csv',header=False,index=False)


if __name__ == "__main__":
	main()
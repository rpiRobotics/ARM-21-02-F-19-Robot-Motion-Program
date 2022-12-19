import matplotlib.pyplot as plt
from general_robotics_toolbox import *
from pandas import *
import sys, traceback
import numpy as np
sys.path.append('../toolbox')
from utils import *
##################################generate equaly divided cartesian path for moveJ and moveL
def gen_curve_normal(seed_vec,curve):
	curve_normal=[]
	error=[]
	for i in range(len(curve)-1):
		moving_direction=curve[i+1]-curve[i]
		moving_direction=moving_direction/np.linalg.norm(moving_direction)
		curve_normal_temp=VectorPlaneProjection(seed_vec,moving_direction)

		curve_normal.append(curve_normal_temp/np.linalg.norm(curve_normal_temp))
		error.append(np.dot(moving_direction,curve_normal[-1]))

	curve_normal.append(curve_normal[-1])
	return np.array(curve_normal)

def gen_curve_normal2(seed_vec,curve):
	moving_direction=curve[1]-curve[0]
	moving_direction=moving_direction/np.linalg.norm(moving_direction)

	curve_normal=[VectorPlaneProjection(seed_vec,moving_direction)]
	prev_direction=moving_direction

	for i in range(1,len(curve)-1):

		new_direction=curve[i+1]-curve[i]
		new_direction=new_direction/np.linalg.norm(new_direction)
		k=np.cross(new_direction,prev_direction)
		k_norm=np.linalg.norm(k)
		theta=np.arcsin(k_norm)
		if theta==0:
			R=np.eye(3)
		else:
			k=k/k_norm
			R=rot(k,theta)
		prev_direction=new_direction

		curve_normal.append(np.dot(R,curve_normal[-1]))


	curve_normal.append(curve_normal[-1])
	return np.array(curve_normal)

def main():
	col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z']
	data = read_csv("from_ge/Curve_in_base_frame.csv", names=col_names)
	curve_x=data['X'].tolist()
	curve_y=data['Y'].tolist()
	curve_z=data['Z'].tolist()
	curve_direction_x=data['direction_x'].tolist()
	curve_direction_y=data['direction_y'].tolist()
	curve_direction_z=data['direction_z'].tolist()
	curve=np.vstack((curve_x, curve_y, curve_z)).T
	curve_direction=np.vstack((curve_direction_x, curve_direction_y, curve_direction_z)).T


	curve_normal=gen_curve_normal([0.97324,	0.091,	-0.21101],curve)


	df=DataFrame({'x':curve[:,0],'y':curve[:,1], 'z':curve[:,2],'x_dir':curve_normal[:,0],'y_dir':curve_normal[:,1], 'z_dir':curve_normal[:,2]})
	df.to_csv('from_ge/Curve_in_base_frame2.csv',header=False,index=False)
	# visualize(curve,curve_direction)
	# d=50
	# curve_backproj=curve-d*curve_normal
	# df=DataFrame({'x':curve_backproj[:,0],'y':curve_backproj[:,1], 'z':curve_backproj[:,2],'x_dir':curve_normal[:,0],'y_dir':curve_normal[:,1], 'z_dir':curve_normal[:,2]})
	# df.to_csv('from_ge/Curve_backproj_in_base_frame2.csv',header=False,index=False)

def rl_traj_gen():
	import glob
	file_list=glob.glob("../rl_fit/train_data/base/*.csv")
	for filename in file_list:
		col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z']
		data = read_csv(filename, names=col_names)
		curve_x=data['X'].tolist()
		curve_y=data['Y'].tolist()
		curve_z=data['Z'].tolist()
		curve=np.vstack((curve_x, curve_y, curve_z)).T
		curve_normal=gen_curve_normal([0.97324,	0.091,	-0.21101],curve)
		df=DataFrame({'x':curve[:,0],'y':curve[:,1], 'z':curve[:,2],'x_dir':curve_normal[:,0],'y_dir':curve_normal[:,1], 'z_dir':curve_normal[:,2]})
		df.to_csv(filename,header=False,index=False)


if __name__ == "__main__":
	main()
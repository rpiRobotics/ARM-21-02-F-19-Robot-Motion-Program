import numpy as np
import sys,copy
sys.path.append('../toolbox')
from toolbox_circular_fit import *
from utils import *

class fitting_toolbox(object):
	def __init__(self,robot1,robot2,curve_js1,curve_js2):
		self.curve_js1=curve_js1
		self.curve_js2=curve_js2
		self.robot1=robot1
		self.robot2=robot2

		###get full curve list
		self.relative_R=[]
		self.relative_path=[]
		self.curve1=[]
		self.curve2=[]
		self.curve_R1=[]
		self.curve_R2=[]
		for i in range(len(curve_js1)):
			pose1_now=robot1.fwd(curve_js1[i])
			pose2_now=robot2.fwd(curve_js2[i])
			# pose2_world_now=robot2.fwd(curve_js2[i],base2_R,base2_p)
			pose2_world_now=robot2.fwd(curve_js2[i],world=True)

			self.curve1.append(pose1_now.p)
			self.curve2.append(pose2_now.p)
			self.curve_R1.append(pose1_now.R)
			self.curve_R2.append(pose2_now.R)

			self.relative_path.append(pose2_world_now.R.T@(pose1_now.p-pose2_world_now.p))
			# self.relative_path.append(np.dot(pose2_world_now.R.T,(pose1_now.p-pose2_world_now.p)))
			self.relative_R.append(pose2_world_now.R.T@pose1_now.R)


		self.relative_R=np.array(self.relative_R)
		self.relative_path=np.array(self.relative_path)
		self.curve1=np.array(self.curve1)
		self.curve2=np.array(self.curve2)
		self.curve_R1=np.array(self.curve_R1)
		self.curve_R2=np.array(self.curve_R2)

		self.curve1_fit=[]
		self.curve1_fit_R=[]
		self.curve1_fit_js=[]

		self.curve2_fit=[]
		self.curve2_fit_R=[]
		self.curve2_fit_js=[]

	def linear_fit(self,data,p_constraint=[]):
		###no constraint
		if len(p_constraint)==0:
			A=np.vstack((np.ones(len(data)),np.arange(0,len(data)))).T
			b=data
			res=np.linalg.lstsq(A,b,rcond=None)[0]
			start_point=res[0]
			slope=res[1].reshape(1,-1)

			data_fit=np.dot(np.arange(0,len(data)).reshape(-1,1),slope)+start_point
		###with constraint point
		else:
			start_point=p_constraint

			A=np.arange(1,len(data)+1).reshape(-1,1)
			b=data-start_point
			res=np.linalg.lstsq(A,b,rcond=None)[0]
			slope=res.reshape(1,-1)

			data_fit=np.dot(np.arange(1,len(data)+1).reshape(-1,1),slope)+start_point

		return data_fit


	def movel_fit(self,curve,curve_js,curve_R,robot,p_constraint=[],R_constraint=[],slope_constraint=[]):	###unit vector slope
		###convert orientation to w first
		curve_w=R2w(curve_R,R_constraint)


		data_fit=self.linear_fit(np.hstack((curve,curve_w)),[] if len(p_constraint)==0 else np.hstack((p_constraint,np.zeros(3))))
		curve_fit=data_fit[:,:3]
		curve_fit_w=data_fit[:,3:]

		curve_fit_R=w2R(curve_fit_w,curve_R[0] if len(R_constraint)==0 else R_constraint)

		p_error=np.linalg.norm(curve-curve_fit,axis=1)

		curve_fit_R=np.array(curve_fit_R)
		ori_error=[]
		for i in range(len(curve)):
			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))
		
		return curve_fit,curve_fit_R,[],np.max(p_error), np.max(ori_error)
		


	def movej_fit(self,curve,curve_js,curve_R,robot,p_constraint=[],R_constraint=[],slope_constraint=[]):
		###convert orientation to w first
		curve_w=R2w(curve_R,R_constraint)


		curve_fit_js=self.linear_fit(curve_js,p_constraint)

		###necessary to fwd every point search to get error calculation
		curve_fit=[]
		curve_fit_R=[]
		for i in range(len(curve_fit_js)):
			pose_temp=robot.fwd(curve_fit_js[i])
			curve_fit.append(pose_temp.p)
			curve_fit_R.append(pose_temp.R)
		curve_fit=np.array(curve_fit)
		curve_fit_R=np.array(curve_fit_R)

		###orientation error
		curve_fit_w=R2w(curve_fit_R,R_constraint)

		p_error=np.linalg.norm(curve-curve_fit,axis=1)

		curve_fit_R=np.array(curve_fit_R)
		ori_error=[]
		for i in range(len(curve)):
			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))
		
		return curve_fit,curve_fit_R,curve_fit_js,np.max(p_error), np.max(ori_error)


	def movec_fit(self,curve,curve_js,curve_R,robot,p_constraint=[],R_constraint=[],slope_constraint=[]):
		curve_w=R2w(curve_R,R_constraint)	

		curve_fit,curve_fit_circle=circle_fit(curve,[] if len(R_constraint)==0 else p_constraint)
		curve_fit_w=self.linear_fit(curve_w,[] if len(R_constraint)==0 else np.zeros(3))

		curve_fit_R=w2R(curve_fit_w,curve_R[0] if len(R_constraint)==0 else R_constraint)

		p_error=np.linalg.norm(curve-curve_fit,axis=1)

		curve_fit_R=np.array(curve_fit_R)
		ori_error=[]
		for i in range(len(curve)):
			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		return curve_fit,curve_fit_R,[],np.max(p_error), np.max(ori_error)

	def get_slope(self,curve_fit,curve_fit_R,breakpoints):
		slope_diff=[]
		slope_diff_ori=[]
		for i in range(1,len(breakpoints)-1):
			slope_diff.append(self.get_angle(curve_fit[breakpoints[i]-1]-curve_fit[breakpoints[i]-2],curve_fit[breakpoints[i]]-curve_fit[breakpoints[i]-1]))

			R_diff_prev=np.dot(curve_fit_R[breakpoints[i]],curve_fit_R[breakpoints[i-1]].T)
			k_prev,theta=R2rot(R_diff_prev)
			R_diff_next=np.dot(curve_fit_R[breakpoints[i+1]-1],curve_fit_R[breakpoints[i]].T)
			k_next,theta=R2rot(R_diff_next)
			slope_diff_ori.append(self.get_angle(k_prev,k_next,less90=True))

		return slope_diff,slope_diff_ori


def main():
	###read in points
	col_names=['X', 'Y', 'Z','direction_x', 'direction_y', 'direction_z'] 
	data = read_csv("../train_data/from_cad/Curve_in_base_frame.csv", names=col_names)
	curve_x=data['X'].tolist()
	curve_y=data['Y'].tolist()
	curve_z=data['Z'].tolist()
	curve=np.vstack((curve_x, curve_y, curve_z)).T


	curve_fit,max_error_all=fit_w_breakpoints(curve,[movel_fit,movec_fit,movec_fit],[0,int(len(curve)/3),int(2*len(curve)/3),len(curve)])

	print(max_error_all)

	fig = plt.figure()
	ax = plt.axes(projection='3d')
	ax.plot3D(curve[:,0], curve[:,1], curve[:,2], 'gray')
	ax.plot3D(curve_fit[:,0], curve_fit[:,1], curve_fit[:,2], 'red')

	plt.show()
if __name__ == "__main__":
	main()

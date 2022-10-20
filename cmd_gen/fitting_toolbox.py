import numpy as np
import sys,copy
sys.path.append('../toolbox')
from toolbox_circular_fit import *
from lambda_calc import *

class fitting_toolbox(object):
	def __init__(self,robot,curve_js,curve=[]):
		###robot: robot class
		###curve_js: points in joint space
		###d: standoff distance
		self.curve_js=curve_js
		self.robot=robot

		###get full orientation list
		if len(curve)>0:
			self.curve_R=[]
			self.curve=curve
			for i in range(len(curve_js)):
				pose_temp=self.robot.fwd(curve_js[i])
				self.curve_R.append(pose_temp.R)
		else:
			self.curve_R=[]
			self.curve=[]
			for i in range(len(curve_js)):
				pose_temp=self.robot.fwd(curve_js[i])
				self.curve_R.append(pose_temp.R)
				self.curve.append(pose_temp.p)

		self.curve_R=np.array(self.curve_R)
		self.curve=np.array(self.curve)

		self.lam=calc_lam_cs(self.curve)

		###seed initial js for inv
		self.q_prev=curve_js[0]

		self.curve_fit=[]
		self.curve_fit_R=[]
		self.curve_fit_js=[]
		self.cartesian_slope_prev=None
		self.js_slope_prev=None

	def R2w(self, curve_R,R_constraint=[]):
		if len(R_constraint)==0:
			R_init=curve_R[0]
			curve_w=[np.zeros(3)]
		else:
			R_init=R_constraint
			R_diff=np.dot(curve_R[0],R_init.T)
			k,theta=R2rot(R_diff)
			k=np.array(k)
			curve_w=[k*theta]
		
		for i in range(1,len(curve_R)):
			R_diff=np.dot(curve_R[i],R_init.T)
			k,theta=R2rot(R_diff)
			k=np.array(k)
			curve_w.append(k*theta)
		return np.array(curve_w)
	def w2R(self,curve_w,R_init):
		curve_R=[]
		for i in range(len(curve_w)):
			theta=np.linalg.norm(curve_w[i])
			if theta==0:
				curve_R.append(R_init)
			else:
				curve_R.append(np.dot(rot(curve_w[i]/theta,theta),R_init))

		return np.array(curve_R)

	def linear_interp(self,p_start,p_end,steps):
		slope=(p_end-p_start)/(steps-1)
		return np.dot(np.arange(0,steps).reshape(-1,1),slope.reshape(1,-1))+p_start

	def orientation_interp(self,R_init,R_end,steps):
		curve_fit_R=[]
		###find axis angle first
		R_diff=np.dot(R_init.T,R_end)
		k,theta=R2rot(R_diff)
		for i in range(steps):
			###linearly interpolate angle
			angle=theta*i/(steps-1)
			R=rot(k,angle)
			curve_fit_R.append(np.dot(R_init,R))
		curve_fit_R=np.array(curve_fit_R)
		return curve_fit_R

	def quatera(self,curve_quat,initial_quat=[]):
		###quaternion regression
		if len(initial_quat)==0:
			Q=np.array(curve_quat).T
			Z=np.dot(Q,Q.T)
			u, s, vh = np.linalg.svd(Z)

			w=np.dot(quatproduct(u[:,1]),quatcomplement(u[:,0]))
			k,theta=q2rot(w)	#get the axis of rotation

			theta1=2*np.arctan2(np.dot(u[:,1],curve_quat[0]),np.dot(u[:,0],curve_quat[0]))
			theta2=2*np.arctan2(np.dot(u[:,1],curve_quat[-1]),np.dot(u[:,0],curve_quat[-1]))

			#get the angle of rotation
			theta=(theta2-theta1)%(2*np.pi)
			if theta>np.pi:
				theta-=2*np.pi

		else:
			###TODO: find better way for orientation continuous constraint 
			curve_quat_cons=np.vstack((curve_quat,np.tile(initial_quat,(999999,1))))
			Q=np.array(curve_quat_cons).T
			Z=np.dot(Q,Q.T)
			u, s, vh = np.linalg.svd(Z)

			w=np.dot(quatproduct(u[:,1]),quatcomplement(u[:,0]))
			k,theta=q2rot(w)

			theta1=2*np.arctan2(np.dot(u[:,1],curve_quat[0]),np.dot(u[:,0],curve_quat[0]))
			theta2=2*np.arctan2(np.dot(u[:,1],curve_quat[-1]),np.dot(u[:,0],curve_quat[-1]))

			#get the angle of rotation
			theta=(theta2-theta1)%(2*np.pi)
			if theta>np.pi:
				theta-=2*np.pi

		curve_fit_R=[]
		R_init=q2R(curve_quat[0])
		
		for i in range(len(curve_quat)):
			###linearly interpolate angle
			angle=theta*i/len(curve_quat)
			R=rot(k,angle)
			curve_fit_R.append(np.dot(R,R_init))
		curve_fit_R=np.array(curve_fit_R)
		return curve_fit_R
	
	def threshold_slope(self,slope_prev,slope,slope_thresh):
		slope_norm=np.linalg.norm(slope)
		slope_prev=slope_prev/np.linalg.norm(slope_prev)
		slope=slope.flatten()/slope_norm

		angle=np.arccos(np.dot(slope_prev,slope))

		if abs(angle)>slope_thresh:
			slope_ratio=np.sin(slope_thresh)/np.sin(abs(angle)-slope_thresh)
			slope_new=slope_prev+slope_ratio*slope
			slope_new=slope_norm*slope_new/np.linalg.norm(slope_new)

			return slope_new/np.linalg.norm(slope_new)

		else:
			return slope

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

	def get_start_slope(self,p1,p2,R1,R2):
		q1=car2js(self.robot,curve_js[0],p1,R1)[0]
		q2=car2js(self.robot,curve_js[0],p2,R2)[0]

		return (q2-q1)/np.linalg.norm(q2-q1)

	def movel_fit(self,curve,curve_js,curve_R,p_constraint=[],R_constraint=[],dqdlam_prev=[], rl=False):	###unit vector slope
		###convert orientation to w first
		curve_w=self.R2w(curve_R,R_constraint)


		data_fit=self.linear_fit(np.hstack((curve,curve_w)),[] if len(p_constraint)==0 else np.hstack((p_constraint,np.zeros(3))))
		curve_fit=data_fit[:,:3]
		curve_fit_w=data_fit[:,3:]

		curve_fit_R=self.w2R(curve_fit_w,curve_R[0] if len(R_constraint)==0 else R_constraint)

		p_error=np.linalg.norm(curve-curve_fit,axis=1)

		curve_fit_R=np.array(curve_fit_R)
		ori_error=[]
		for i in range(len(curve)):
			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		# ###slope thresholding 1
		# if len(dqdlam_prev)>0:
		# 	dqdlam_prev=dqdlam_prev/np.linalg.norm(dqdlam_prev)
		# 	slope_cur=self.get_start_slope(self.curve_fit[-1],curve_fit[0],self.curve_fit_R[-1],curve_fit_R[0])
		# 	if get_angle(slope_cur,dqdlam_prev)>self.slope_constraint:
		# 		# print('triggering slope threshold')
		# 		slope_new=self.threshold_slope(dqdlam_prev,slope_cur,self.slope_constraint)
		# 		###propogate to L slope
		# 		J=self.robot.jacobian(self.curve_fit_js[-1])
		# 		nu=J@slope_new
		# 		p_slope=nu[3:]/np.linalg.norm(nu[3:])
		# 		w_slope=nu[:3]/np.linalg.norm(nu[:3])
		# 		###find correct length
		# 		lam_p=p_slope@(curve[-1]-p_constraint)
		# 		lam_p_all=np.linspace(0,lam_p,num=len(curve)+1)[1:].reshape(-1,1)
		# 		lam_w=w_slope@(curve_w[-1])
		# 		lam_w_all=np.linspace(0,lam_w,num=len(curve)+1)[1:].reshape(-1,1)
		# 		#position

		# 		curve_fit=lam_p_all@p_slope.reshape(1,-1)+p_constraint
		# 		p_error=np.linalg.norm(curve-curve_fit,axis=1)
		# 		#orientation
		# 		curve_fit_w=lam_w_all@w_slope.reshape(1,-1)
		# 		curve_fit_R=self.w2R(curve_fit_w,R_constraint)
		# 		ori_error=[]
		# 		for i in range(len(curve)):
		# 			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		###slope thresholding 2
		if len(dqdlam_prev)>0:
			q1=car2js(self.robot,curve_js[0],curve_fit[0],curve_fit_R[0])[0]
			dqdlam_cur=(q1-self.curve_fit_js[-1])/(self.lam[len(self.curve_fit_js)]-self.lam[len(self.curve_fit_js)-1])
			# print(np.max(np.abs(dqdlam_cur-dqdlam_prev)))
			if np.max(np.abs(dqdlam_cur-dqdlam_prev))>self.dqdlam_slope:
				dqdlam_new=np.clip(dqdlam_cur,dqdlam_prev-self.dqdlam_slope,dqdlam_prev+self.dqdlam_slope)
				print('triggering slope threshold')
				print('fitting slope:',dqdlam_cur)
				print('lower bound  :',dqdlam_prev-self.dqdlam_slope)
				print('upper bound  :',dqdlam_prev+self.dqdlam_slope)
				print('output slope :',dqdlam_new)
				slope_new=dqdlam_new/np.linalg.norm(dqdlam_new)
				###propogate to L slope
				J=self.robot.jacobian(self.curve_fit_js[-1])
				nu=J@slope_new
				p_slope=nu[3:]/np.linalg.norm(nu[3:])
				w_slope=nu[:3]/np.linalg.norm(nu[:3])
				###find correct length
				lam_p=p_slope@(curve[-1]-p_constraint)
				lam_p_all=np.linspace(0,lam_p,num=len(curve)+1)[1:].reshape(-1,1)
				lam_w=w_slope@(curve_w[-1])
				lam_w_all=np.linspace(0,lam_w,num=len(curve)+1)[1:].reshape(-1,1)
				#position
				curve_fit=lam_p_all@p_slope.reshape(1,-1)+p_constraint
				p_error=np.linalg.norm(curve-curve_fit,axis=1)
				#orientation
				curve_fit_w=lam_w_all@w_slope.reshape(1,-1)
				curve_fit_R=self.w2R(curve_fit_w,R_constraint)
				ori_error=[]
				for i in range(len(curve)):
					ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		if rl:
			return curve_fit,curve_fit_R,[], p_error, ori_error

		return curve_fit,curve_fit_R,[],np.max(p_error), np.max(ori_error)
		


	def movej_fit(self,curve,curve_js,curve_R,p_constraint=[],R_constraint=[],dqdlam_prev=[], rl=False):
		curve_fit_js=self.linear_fit(curve_js,p_constraint)

		###necessary to fwd every point search to get error calculation
		curve_fit=[]
		curve_fit_R=[]
		for i in range(len(curve_fit_js)):
			pose_temp=self.robot.fwd(curve_fit_js[i])
			curve_fit.append(pose_temp.p)
			curve_fit_R.append(pose_temp.R)
		curve_fit=np.array(curve_fit)
		curve_fit_R=np.array(curve_fit_R)

		###error
		p_error=np.linalg.norm(curve-curve_fit,axis=1)
		curve_fit_R=np.array(curve_fit_R)
		ori_error=[]
		for i in range(len(curve)):
			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		###slope thresholding 1
		# if len(dqdlam_prev)>0:
		# 	dqdlam_prev=dqdlam_prev/np.linalg.norm(dqdlam_prev)
		# 	slope_cur=self.get_start_slope(self.curve_fit[-1],curve_fit[0],self.curve_fit_R[-1],curve_fit_R[0])
		# 	if get_angle(slope_cur,dqdlam_prev)>self.slope_constraint:
		# 		slope_new=self.threshold_slope(dqdlam_prev,slope_cur,self.slope_constraint)
		# 		###find correct length
		# 		lam_js=slope_new@(curve_js[-1]-p_constraint)
		# 		lam_js_all=np.linspace(0,lam_js,num=len(curve)+1)[1:].reshape(-1,1)
		# 		#joints propogation
		# 		curve_fit_js=lam_js_all@slope_new.reshape(1,-1)+p_constraint
		# 		curve_fit=[]
		# 		curve_fit_R=[]
		# 		for i in range(len(curve_fit_js)):
		# 			pose_temp=self.robot.fwd(curve_fit_js[i])
		# 			curve_fit.append(pose_temp.p)
		# 			curve_fit_R.append(pose_temp.R)
		# 		curve_fit=np.array(curve_fit)
		# 		curve_fit_R=np.array(curve_fit_R)

		# 		###error
		# 		p_error=np.linalg.norm(curve-curve_fit,axis=1)
		# 		curve_fit_R=np.array(curve_fit_R)
		# 		ori_error=[]
		# 		for i in range(len(curve)):
		# 			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))
		
		###slope thresholding 2
		if len(dqdlam_prev)>0:
			q1=curve_fit_js[0]
			dqdlam_cur=(q1-self.curve_fit_js[-1])/(self.lam[len(self.curve_fit_js)]-self.lam[len(self.curve_fit_js)-1])

			if np.max(np.abs(dqdlam_cur-dqdlam_prev))>self.dqdlam_slope:

				dqdlam_new=np.clip(dqdlam_cur,dqdlam_prev-self.dqdlam_slope,dqdlam_prev+self.dqdlam_slope)
				print('triggering slope threshold')
				slope_new=dqdlam_new/np.linalg.norm(dqdlam_new)
				###find correct length
				lam_js=slope_new@(curve_js[-1]-p_constraint)
				lam_js_all=np.linspace(0,lam_js,num=len(curve)+1)[1:].reshape(-1,1)
				#joints propogation
				curve_fit_js=lam_js_all@slope_new.reshape(1,-1)+p_constraint
				curve_fit=[]
				curve_fit_R=[]
				for i in range(len(curve_fit_js)):
					pose_temp=self.robot.fwd(curve_fit_js[i])
					curve_fit.append(pose_temp.p)
					curve_fit_R.append(pose_temp.R)
				curve_fit=np.array(curve_fit)
				curve_fit_R=np.array(curve_fit_R)

				###error
				p_error=np.linalg.norm(curve-curve_fit,axis=1)
				curve_fit_R=np.array(curve_fit_R)
				ori_error=[]
				for i in range(len(curve)):
					ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		if rl:
			return curve_fit,curve_fit_R,curve_fit_js, p_error, ori_error

		return curve_fit,curve_fit_R,curve_fit_js,np.max(p_error), np.max(ori_error)


	def movec_fit(self,curve,curve_js,curve_R,p_constraint=[],R_constraint=[],dqdlam_prev=[], rl=False):
		curve_w=self.R2w(curve_R,R_constraint)	

		curve_fit,curve_fit_circle=circle_fit(curve,[] if len(R_constraint)==0 else p_constraint)
		curve_fit_w=self.linear_fit(curve_w,[] if len(R_constraint)==0 else np.zeros(3))

		curve_fit_R=self.w2R(curve_fit_w,curve_R[0] if len(R_constraint)==0 else R_constraint)

		p_error=np.linalg.norm(curve-curve_fit,axis=1)

		curve_fit_R=np.array(curve_fit_R)
		ori_error=[]
		for i in range(len(curve)):
			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		###slope thresholding 1
		# if len(dqdlam_prev)>0:
		# 	dqdlam_prev=dqdlam_prev/np.linalg.norm(dqdlam_prev)
		# 	# print('triggering slope threshold')
		# 	slope_cur=self.get_start_slope(self.curve_fit[-1],curve_fit[0],self.curve_fit_R[-1],curve_fit_R[0])
		# 	if get_angle(slope_cur,dqdlam_prev)>self.slope_constraint:
		# 		slope_new=self.threshold_slope(dqdlam_prev,slope_cur,self.slope_constraint)
		# 		###propogate to L slope
		# 		J=self.robot.jacobian(self.curve_fit_js[-1])
		# 		nu=J@slope_new
		# 		p_slope=nu[3:]/np.linalg.norm(nu[3:])
		# 		w_slope=nu[:3]/np.linalg.norm(nu[:3])
		# 		###position
		# 		curve_fit,curve_fit_circle=circle_fit_w_slope1(curve,p_constraint,p_slope)
		# 		p_error=np.linalg.norm(curve-curve_fit,axis=1)
		# 		lam_w=w_slope@(curve_w[-1])
		# 		lam_w_all=np.linspace(0,lam_w,num=len(curve)+1)[1:].reshape(-1,1)
		# 		###orientation
		# 		curve_fit_w=lam_w_all@w_slope.reshape(1,-1)
		# 		curve_fit_R=self.w2R(curve_fit_w,R_constraint)
		# 		ori_error=[]
		# 		for i in range(len(curve)):
		# 			ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		###slope thresholding 2
		if len(dqdlam_prev)>0:
			q1=car2js(self.robot,curve_js[0],curve_fit[0],curve_fit_R[0])[0]
			dqdlam_cur=(q1-self.curve_fit_js[-1])/(self.lam[len(self.curve_fit_js)]-self.lam[len(self.curve_fit_js)-1])
			
			if np.max(np.abs(dqdlam_cur-dqdlam_prev))>self.dqdlam_slope:

				dqdlam_new=np.clip(dqdlam_cur,dqdlam_prev-self.dqdlam_slope,dqdlam_prev+self.dqdlam_slope)
				print('triggering slope threshold')
				slope_new=dqdlam_new/np.linalg.norm(dqdlam_new)
				###propogate to L slope
				J=self.robot.jacobian(self.curve_fit_js[-1])
				nu=J@slope_new
				p_slope=nu[3:]/np.linalg.norm(nu[3:])
				w_slope=nu[:3]/np.linalg.norm(nu[:3])
				###position
				curve_fit,curve_fit_circle=circle_fit_w_slope1(curve,p_constraint,p_slope)
				p_error=np.linalg.norm(curve-curve_fit,axis=1)
				lam_w=w_slope@(curve_w[-1])
				lam_w_all=np.linspace(0,lam_w,num=len(curve)+1)[1:].reshape(-1,1)
				###orientation
				curve_fit_w=lam_w_all@w_slope.reshape(1,-1)
				curve_fit_R=self.w2R(curve_fit_w,R_constraint)
				ori_error=[]
				for i in range(len(curve)):
					ori_error.append(get_angle(curve_R[i,:,-1],curve_fit_R[i,:,-1]))

		if rl:
			return curve_fit,curve_fit_R,[], p_error, ori_error

		return curve_fit,curve_fit_R,[],np.max(p_error), np.max(ori_error)

	def get_slope(self,curve_fit,curve_fit_R,breakpoints):
		slope_diff=[]
		slope_diff_ori=[]
		for i in range(1,len(breakpoints)-1):
			slope_diff.append(get_angle(curve_fit[breakpoints[i]-1]-curve_fit[breakpoints[i]-2],curve_fit[breakpoints[i]]-curve_fit[breakpoints[i]-1]))

			R_diff_prev=np.dot(curve_fit_R[breakpoints[i]],curve_fit_R[breakpoints[i-1]].T)
			k_prev,theta=R2rot(R_diff_prev)
			R_diff_next=np.dot(curve_fit_R[breakpoints[i+1]-1],curve_fit_R[breakpoints[i]].T)
			k_next,theta=R2rot(R_diff_next)
			slope_diff_ori.append(get_angle(k_prev,k_next,less90=True))

		return slope_diff,slope_diff_ori

	def get_slope_js(self,curve_fit_js,breakpoints):
		slope_diff_js=[]

		for i in range(1,len(breakpoints)-1):
			slope_diff_js.append(get_angle(curve_fit_js[breakpoints[i]-1]-curve_fit_js[breakpoints[i]-2],curve_fit_js[breakpoints[i]]-curve_fit_js[breakpoints[i]-1]))

		return slope_diff_js

	# def get_slope_js(self, curve_fit_js, breakpoints):
	# 	slope_diff_js = []
	#
	# 	for i in range(1, len(breakpoints) - 1):
	# 		slope1 = curve_fit_js[breakpoints[i] - 1] - curve_fit_js[breakpoints[i] - 2]
	# 		slope2 = curve_fit_js[breakpoints[i]] - curve_fit_js[breakpoints[i] - 1]
	# 		slope_diff = np.abs(slope1 - slope2)
	# 		# slope_diff = np.degrees(slope_diff)
	# 		# slope_diff = ["{:.3f}".format(slope_diff[i]) for i in range(len(slope_diff))]
	# 		# slope_diff_js.append("\t".join(slope_diff))
	# 		slope_diff_js.append(slope_diff)
	# 	# ret = "\n".join(slope_diff_js)
	# 	ret = np.array(slope_diff_js)
	# 	return ret


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

import numpy as np
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt
from pandas import *
from .fitting_toolbox import *
import sys


#####################3d curve-fitting with MoveL, MoveJ, MoveC; stepwise incremental bi-section searched self.breakpoints###############################

class greedy_fit(fitting_toolbox):
	def __init__(self,robot,curve_js,min_length,max_error_threshold,max_ori_threshold=np.radians(3)):
		super().__init__(robot,curve_js[:])
		self.max_error_threshold=max_error_threshold
		self.max_ori_threshold=max_ori_threshold
		self.step=int(len(curve_js)/25)
		self.c_min_length=50
		self.min_step=int(min_length/np.average(np.diff(self.lam)))
		self.min_step_start_end=200

		self.slope_constraint=np.radians(360)
		self.dqdlam_slope=999
		self.break_early=False
		###initial primitive candidates
		self.primitives={'movel_fit':self.movel_fit_greedy,'movej_fit':self.movej_fit_greedy,'movec_fit':self.movec_fit_greedy}

	def movel_fit_greedy(self,curve,curve_js,curve_R, rl=False):	###unit vector slope
		
		return self.movel_fit(curve,curve_js,curve_R,self.curve_fit[-1] if len(self.curve_fit)>0 else [],self.curve_fit_R[-1] if len(self.curve_fit_R)>0 else [], dqdlam_prev=(self.curve_fit_js[-1]-self.curve_fit_js[-2])/(self.lam[len(self.curve_fit_js)-1]-self.lam[len(self.curve_fit_js)-2]) if len(self.curve_fit_js)>1 else [], rl=rl)
	


	def movej_fit_greedy(self,curve,curve_js,curve_R, rl=False):

		return self.movej_fit(curve,curve_js,curve_R,self.curve_fit_js[-1] if len(self.curve_fit_js)>0 else [], dqdlam_prev=(self.curve_fit_js[-1]-self.curve_fit_js[-2])/(self.lam[len(self.curve_fit_js)-1]-self.lam[len(self.curve_fit_js)-2]) if len(self.curve_fit_js)>1 else [], rl=rl)


	def movec_fit_greedy(self,curve,curve_js,curve_R, rl=False):
		return self.movec_fit(curve,curve_js,curve_R,self.curve_fit[-1] if len(self.curve_fit)>0 else [],self.curve_fit_R[-1] if len(self.curve_fit_R)>0 else [], dqdlam_prev=(self.curve_fit_js[-1]-self.curve_fit_js[-2])/(self.lam[len(self.curve_fit_js)-1]-self.lam[len(self.curve_fit_js)-2]) if len(self.curve_fit_js)>1 else [], rl=rl)

	def bisect(self,primitive,cur_idx, rl=False):

		next_point = min(self.step,len(self.curve)-self.breakpoints[-1])
		prev_point=0
		prev_possible_point=0

		while True:
			###end condition, bisection bp converges
			if next_point==prev_point:
				if rl:
					if np.max(max_error)<self.max_error_threshold and np.max(max_ori_error)<self.max_ori_threshold:
						return curve_fit,curve_fit_R,curve_fit_js,max_error,max_ori_error
					else:
						next_point=max(prev_possible_point,2)
						return primitive(self.curve[cur_idx:cur_idx+next_point],self.curve_js[cur_idx:cur_idx+next_point],self.curve_R[cur_idx:cur_idx+next_point], rl=rl)
				else:
					if max_error<self.max_error_threshold and max_ori_error<self.max_ori_threshold:
						return curve_fit,curve_fit_R,curve_fit_js,max_error,max_ori_error
					else:
						next_point=max(prev_possible_point,2)
						return primitive(self.curve[cur_idx:cur_idx+next_point],self.curve_js[cur_idx:cur_idx+next_point],self.curve_R[cur_idx:cur_idx+next_point], rl=rl)
			###fitting
			curve_fit,curve_fit_R,curve_fit_js,max_error,max_ori_error=primitive(self.curve[cur_idx:cur_idx+next_point],self.curve_js[cur_idx:cur_idx+next_point],self.curve_R[cur_idx:cur_idx+next_point], rl=rl)

			###bp going backward to meet threshold
			if rl:
				if np.max(max_error) > self.max_error_threshold or np.max(max_ori_error) > self.max_ori_threshold:
					prev_point_temp = next_point
					next_point -= int(np.abs(next_point - prev_point) / 2)
					prev_point = prev_point_temp

				###bp going forward to get close to threshold
				else:
					prev_possible_point = next_point
					prev_point_temp = next_point
					next_point = min(next_point + int(np.abs(next_point - prev_point)), len(self.curve) - cur_idx)
					prev_point = prev_point_temp
			else:
				if max_error>self.max_error_threshold or max_ori_error>self.max_ori_threshold:
					prev_point_temp=next_point
					next_point-=int(np.abs(next_point-prev_point)/2)
					prev_point=prev_point_temp

				###bp going forward to get close to threshold
				else:
					prev_possible_point=next_point
					prev_point_temp=next_point
					next_point= min(next_point + int(np.abs(next_point-prev_point)),len(self.curve)-cur_idx)
					prev_point=prev_point_temp


	def fit_under_error(self):

		###initialize
		self.breakpoints=[0]
		primitives=[]
		p_bp=[]
		q_bp=[]

		self.curve_fit=[]
		self.curve_fit_R=[]
		self.curve_fit_js=[]

		while self.breakpoints[-1]<len(self.curve)-1:

			max_errors={}
			max_ori_errors={}
			length={}
			curve_fit={}
			curve_fit_R={}
			curve_fit_js={}

			###bisection search for each primitive 
			for key in self.primitives: 
				curve_fit[key],curve_fit_R[key],curve_fit_js[key],max_errors[key],max_ori_errors[key]=self.bisect(self.primitives[key],self.breakpoints[-1])
				length[key]=len(curve_fit[key])
			###find best primitive
			if length['movec_fit']==length['movel_fit'] and length['movel_fit']==length['movej_fit']:
				key=min(max_errors, key=max_errors.get)
			else:
				key=max(length, key=length.get)

			###moveC length thresholding (>50mm)
			if key=='movec_fit' and np.linalg.norm(curve_fit['movec_fit'][-1]-curve_fit['movec_fit'][0])<self.c_min_length:
				key='movel_fit'


			
			primitives.append(key)
			self.breakpoints.append(min(self.breakpoints[-1]+len(curve_fit[key]),len(self.curve)))
			self.curve_fit.extend(curve_fit[key])
			self.curve_fit_R.extend(curve_fit_R[key])

			if key=='movej_fit':
				self.curve_fit_js.extend(curve_fit_js[key])
			else:
				###inv here to save time
				if len(self.curve_fit_js)>1:
					q_init=self.curve_fit_js[-1]
				else:
					q_init=self.curve_js[0]

				curve_fit_js=car2js(self.robot,q_init,curve_fit[key],curve_fit_R[key])
			
				self.curve_fit_js.extend(curve_fit_js)

			if key=='movec_fit':
				p_bp.append([curve_fit[key][int(len(curve_fit[key])/2)],curve_fit[key][-1]])
				q_bp.append([curve_fit_js[int(len(curve_fit_R[key])/2)],curve_fit_js[-1]])
			elif key=='movel_fit':
				p_bp.append([curve_fit[key][-1]])
				q_bp.append([curve_fit_js[-1]])
			else:
				p_bp.append([curve_fit[key][-1]])
				q_bp.append([curve_fit_js[key][-1]])


			print(self.breakpoints)
			print(primitives)
			print(max_errors[key],max_ori_errors[key])
			
		self.curve_fit=np.array(self.curve_fit)
		self.curve_fit_R=np.array(self.curve_fit_R)
		self.curve_fit_js=np.array(self.curve_fit_js)

		return np.array(self.breakpoints),primitives,p_bp,q_bp

	def merge_bp(self,breakpoints,primitives,p_bp,q_bp):
		###merge closely programmed bp's
		bp_diff=np.diff(breakpoints)
		close_indices=np.argwhere(bp_diff<self.min_step).flatten().astype(int)
		#remove first and last bp if there
		if 0 in close_indices and breakpoints[1]>self.min_step_start_end:
			close_indices=close_indices[1:]
		if len(breakpoints)-2 in close_indices:
			close_indices=close_indices[:-1]

		#multi-merging
		remove_idx=[]
		for i in range(len(close_indices)-1):
			if close_indices[i+1] - close_indices[i]==1:
				for m in range(i+1,len(close_indices)-1):
					if breakpoints[close_indices[m]]-breakpoints[close_indices[i]]<self.min_step:
						remove_idx.append(m)
					else:
						break
				i=remove_idx[-1]+1
		close_indices=np.delete(close_indices,remove_idx)

		###remove old breakpoints
		indicies2remove=(close_indices+1).tolist()
		indicies2remove.reverse()
		for i in indicies2remove:
			del breakpoints[i]
			del primitives[i]
			del p_bp[i]
			del q_bp[i]

		#second last point removal
		if breakpoints[-1]-breakpoints[-2]<self.min_step_start_end:
			if primitives[-2]=='movej_fit':
				curve_fit,curve_fit_R,curve_fit_js,_,_=self.movej_fit(curve[breakpoints[-2]:],curve_js[breakpoints[-2]:],curve_R[breakpoints[-2]:],p_constraint=points[-2][-1],R_constraint=self.robot.fwd(q_bp[-2][-1]).R)
				points[-2][-1]=curve_fit[-1]
				q_bp[-2][-1]=curve_fit_js[-1]
			elif primitives[-2]=='movel_fit':
				curve_fit,curve_fit_R,_,_,_=self.movel_fit(curve[breakpoints[-2]:],curve_js[breakpoints[-2]:],curve_R[breakpoints[-2]:],p_constraint=points[-2][-1],R_constraint=self.robot.fwd(q_bp[-2][-1]).R)
				points[-2][-1]=curve_fit[-1]
				q_bp[-2][-1]=car2js(self.robot,self.curve_fit_js[breakpoints[-1]],p_bp[-2][-1],curve_fit_R[-1])[0]
			else:
				curve_fit,curve_fit_R,_,_,_=self.movec_fit(curve[breakpoints[-2]:],curve_js[breakpoints[-2]:],curve_R[breakpoints[-2]:],p_constraint=points[-2][-1],R_constraint=self.robot.fwd(q_bp[-2][-1]).R)
				points[-2][0]=curve_fit[int(len(curve_fit)/2)]
				points[-2][-1]=curve_fit[-1]
				q_bp[-2][0]=car2js(self.robot,self.curve_fit_js[breakpoints[-2]],p_bp[-2][0],curve_fit_R[int(len(curve_fit)/2)])[0]
				q_bp[-2][-1]=car2js(self.robot,self.curve_fit_js[breakpoints[-1]],p_bp[-2][-1],curve_fit_R[-1])[0]

			del breakpoints[-1]
			del primitives[-1]
			del p_bp[-1]
			del q_bp[-1]



		return breakpoints,primitives,p_bp,q_bp

	def merge_bp2(self,breakpoints,primitives,p_bp,q_bp,d=10):

		#merging breakpoints within d
		p_bp_np=np.array([item[-1] for item in p_bp])
		bp_diff=np.linalg.norm(np.diff(p_bp_np,axis=0),axis=1)
		close_indices=np.argwhere(bp_diff<d).flatten().astype(int)

		close_indices=close_indices+1### delete later one
		#remove first and last bp if there
		if 1 in close_indices:
			close_indices=close_indices[1:]
		if len(breakpoints)-1 in close_indices:
			close_indices[-1]=len(breakpoints)-2


		#multi-merging
		remove_idx=[]

		i=0
		while i<len(close_indices)-1:
			if close_indices[i+1] - close_indices[i]==1:
				###remove last of consecutive bp's that's greater than d
				for m in range(i+2,len(close_indices)-1):
					if np.linalg.norm(p_bp[close_indices[m]][-1]-p_bp[close_indices[i]][-1])>d:
						remove_idx.append(m)
						i=remove_idx[-1]
						break
			i+=1

		close_indices=np.delete(close_indices,remove_idx)

		breakpoints=np.delete(breakpoints,close_indices)
		primitives=np.delete(primitives,close_indices).tolist()
		q_bp=np.delete(q_bp,close_indices).tolist()
		p_bp=np.delete(p_bp,close_indices).tolist()

		return breakpoints,primitives,p_bp,q_bp



def main():
	dataset='wood/'
	solution_dir='baseline/'
	data_dir="../data/"+dataset+solution_dir

	###read in points
	# curve_js = read_csv("../train_data/wood/Curve_js.csv",header=None).values
	curve_js = read_csv(data_dir+'Curve_js.csv',header=None).values

	robot=abb6640(d=50)

	max_error_threshold=0.3
	min_length=10
	greedy_fit_obj=greedy_fit(robot,curve_js, min_length=min_length,max_error_threshold=max_error_threshold)

	###set primitive choices, defaults are all 3
	# greedy_fit_obj.primitives={'movel_fit':greedy_fit_obj.movel_fit_greedy,'movec_fit':greedy_fit_obj.movec_fit_greedy}

	# greedy_fit_obj.primitives={'movej_fit':greedy_fit_obj.movej_fit_greedy}
	# greedy_fit_obj.primitives={'movel_fit':greedy_fit_obj.movel_fit_greedy}
	# greedy_fit_obj.primitives={'movec_fit':greedy_fit_obj.movec_fit_greedy}

	# greedy_fit_obj.primitives={'movel_fit':greedy_fit_obj.movel_fit_greedy,'movej_fit':greedy_fit_obj.movej_fit_greedy}

	breakpoints,primitives,p_bp,q_bp=greedy_fit_obj.fit_under_error()
	print('slope diff js (deg): ', greedy_fit_obj.get_slope_js(greedy_fit_obj.curve_fit_js,breakpoints))
	
	############insert initial configuration#################
	primitives.insert(0,'moveabsj_fit')
	p_bp.insert(0,[greedy_fit_obj.curve_fit[0]])
	q_bp.insert(0,[greedy_fit_obj.curve_fit_js[0]])


	# breakpoints,primitives,p_bp,q_bp=greedy_fit_obj.merge_bp(breakpoints,primitives,p_bp,q_bp)
	# print(breakpoints)
	###plt
	###3D plot
	plt.figure()
	ax = plt.axes(projection='3d')
	ax.plot3D(greedy_fit_obj.curve[:,0], greedy_fit_obj.curve[:,1],greedy_fit_obj.curve[:,2], 'gray',label='original')
	
	ax.plot3D(greedy_fit_obj.curve_fit[:,0], greedy_fit_obj.curve_fit[:,1], greedy_fit_obj.curve_fit[:,2],'green',label='fitting')
	plt.legend()
	plt.show()

	###adjust breakpoint index
	breakpoints[1:]=breakpoints[1:]-1

	print(len(breakpoints))
	print(len(primitives))
	print(len(p_bp))
	print(len(q_bp))

	df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
	df.to_csv('greedy_output/command.csv',header=True,index=False)
	df=DataFrame({'x':greedy_fit_obj.curve_fit[:,0],'y':greedy_fit_obj.curve_fit[:,1],'z':greedy_fit_obj.curve_fit[:,2],\
		'R1':greedy_fit_obj.curve_fit_R[:,0,0],'R2':greedy_fit_obj.curve_fit_R[:,0,1],'R3':greedy_fit_obj.curve_fit_R[:,0,2],\
		'R4':greedy_fit_obj.curve_fit_R[:,1,0],'R5':greedy_fit_obj.curve_fit_R[:,1,1],'R6':greedy_fit_obj.curve_fit_R[:,1,2],\
		'R7':greedy_fit_obj.curve_fit_R[:,2,0],'R8':greedy_fit_obj.curve_fit_R[:,2,1],'R9':greedy_fit_obj.curve_fit_R[:,2,2]})
	df.to_csv('greedy_output/curve_fit.csv',header=True,index=False)
	DataFrame(greedy_fit_obj.curve_fit_js).to_csv('greedy_output/curve_fit_js.csv',header=False,index=False)


def merging():
	dataset='wood/'
	solution_dir='curve_pose_opt7/'
	data_dir="../data/"+dataset+solution_dir

	###read in points
	# curve_js = read_csv("../train_data/wood/Curve_js.csv",header=None).values
	curve_js = read_csv(data_dir+'Curve_js.csv',header=None).values

	robot=abb6640(d=50)

	max_error_threshold=0.02
	min_length=8			##mm
	greedy_fit_obj=greedy_fit(robot,curve_js, min_length=min_length,max_error_threshold=max_error_threshold)


	cmd_dir='greedy_output/'

	ms = MotionSend()
	breakpoints,primitives,p_bp,q_bp=ms.extract_data_from_cmd(cmd_dir+'command.csv')

	breakpoints,primitives,p_bp,q_bp=greedy_fit_obj.merge_bp2(breakpoints,primitives,p_bp,q_bp,d=min_length)

	df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
	df.to_csv('greedy_output/command_merged.csv',header=True,index=False)

if __name__ == "__main__":
	# merging()
	# greedy_execute()
	main()

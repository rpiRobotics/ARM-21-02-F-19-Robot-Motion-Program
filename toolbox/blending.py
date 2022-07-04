from pandas import read_csv, DataFrame
import sys, copy
sys.path.append('data/')
sys.path.append('toolbox/')
from toolbox_circular_fit import *
from abb_motion_program_exec_client import *
from robots_def import *
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline, BPoly
from lambda_calc import *
from math import ceil

def form_traj_from_bp(q_bp,primitives,robot):
	###TODO: generate equally spaced curve
	N=100#number in each segment
	curve_js=[q_bp[0][0]]
	init_pose=robot.fwd(q_bp[0][0])
	curve=[init_pose.p]
	curve_R=[init_pose.R]
	breakpoints=[0]
	for i in range(1,len(primitives)):
		if primitives[i]=='movej_fit':
			#interpolate joint space linearly
			q_movej=np.linspace(curve_js[-1],q_bp[i][0],num=N+1)[1:]
			curve_js.extend(q_movej)
			#propogate to cartesian space
			for q in q_movej:
				pose_temp=robot.fwd(q)
				curve.append(pose_temp.p)
				curve_R.append(pose_temp.R)
		elif primitives[i]=='movel_fit':
			pose_end=robot.fwd(q_bp[i][0])
			#interpolate xyz linearly
			p_movel=np.linspace(curve[-1],pose_end.p,num=N+1)[1:]
			curve.extend(p_movel)
			#interpolate orientation linearly
			R_movel_end=pose_end.R@curve_R[-1].T
			k,theta=R2rot(R_movel_end)
			theta_movel=np.linspace(0,theta,num=N+1)[1:]
			#propogate to joint space
			R_movel=[]
			for j in range(len(p_movel)):
				R_temp=rot(k,theta_movel[j])@curve_R[-1]
				R_movel.append(R_temp)
				curve_js.append(car2js(robot,curve_js[-1],p_movel[j],R_temp)[0])
			curve_R.extend(R_movel)
		else:
			pose_mid=robot.fwd(q_bp[i][0])
			pose_end=robot.fwd(q_bp[i][1])
			p_movec=arc_from_3point(curve[-1],pose_end.p,pose_mid.p,N+1)[1:]
			curve.extend(p_movec)
			#interpolate orientation linearly
			R_movec_end=pose_end.R@curve_R[-1].T
			k,theta=R2rot(R_movec_end)
			theta_movel=np.linspace(0,theta,num=N+1)[1:]
			#propogate to joint space
			R_movec=[]
			for j in range(len(p_movec)):
				R_temp=rot(k,theta_movel[j])@curve_R[-1]
				R_movec.append(R_temp)
				curve_js.append(car2js(robot,curve_js[-1],p_movec[j],R_temp)[0])
			curve_R.extend(R_movec)

		breakpoints.append(len(curve_js)-1)

	return np.array(curve), np.array(curve_R), np.array(curve_js), np.array(breakpoints)


def blend_js_from_primitive(curve, curve_js, breakpoints, primitives,robot,zone=10):
	###blending in joint space with known primitives
	#curve:			curve cartesian configuration
	#curve_js:		curve joint configuration
	#primitives:	primitive choices, L,C,J
	#breakpoints:	breakpoint index in curve&curve_js
	#robot:			robot kin tool def
	#zone:			blending zone
	##return
	#curve_blended: blended trajectory
	#curve_blended_js: blended trajectory in joint space

	lam=calc_lam_cs(curve)
	blending_start_idx=[]
	blending_end_idx=[0]
	###determine actual blending zone to avoid overlapping
	for i in range(1,len(breakpoints)-1):
		if lam[breakpoints[i]] - zone < lam[blending_end_idx[-1]]:	#if start overlap
			blending_start_idx.append(blending_end_idx[-1]+1)
		else:
			blending_start_idx.append(np.argmin(np.abs(lam-lam[breakpoints[i]]+zone)))

		if lam[breakpoints[i]]+zone > (lam[breakpoints[i]]+lam[breakpoints[i+1]])/2:	#if end overlap
			blending_end_idx.append(int((breakpoints[i]+breakpoints[i+1])/2)-1)
		else:
			blending_end_idx.append(np.argmin(np.abs(lam-lam[breakpoints[i]]-zone)))

	blending_end_idx.pop(0)	#remove first one

	curve_js_blended=blend_js2(curve_js,breakpoints,lam,blending_start_idx,blending_end_idx)
	curve_blended=[]
	curve_R_blended=[]
	for q in curve_js_blended:
		pose_temp=robot.fwd(q)
		curve_blended.append(pose_temp.p)
		curve_R_blended.append(pose_temp.R)
	return np.array(curve_js_blended),np.array(curve_blended),np.array(curve_R_blended)


def blend_js2(curve_js,breakpoints,lam,blending_start_idx,blending_end_idx):
	#curve_js: 			curve in joint space
	#breakpoints:		breakpoints
	#lam: 				path length
	#blending_start_idx:		blending start idx
	#blending_end_idx:		blending end idx
	##return
	#curve_js_blended: 	blended trajectory with movej

	curve_js_blended=copy.deepcopy(curve_js)
	skip=False
	merged_idx=[]

	for i in range(1,len(breakpoints)-1):

		start_idx = blending_start_idx[i-1]
		end_idx	= blending_end_idx[i-1]

		for j in range(len(curve_js[0])):
			poly = BPoly.from_derivatives([lam[start_idx],lam[end_idx]], [[curve_js[start_idx,j],(curve_js[start_idx,j]-curve_js[start_idx-1,j])/(lam[start_idx]-lam[start_idx-1])], [curve_js[end_idx,j],(curve_js[end_idx+1,j]-curve_js[end_idx,j])/(lam[end_idx+1]-lam[end_idx])]])
			curve_js_blended[start_idx:end_idx,j]=poly(lam[start_idx:end_idx])

	return curve_js_blended

def blend_cart_from_primitive(curve,curve_R, curve_js, breakpoints, primitives,robot,ldot):

	curve_blend = copy.deepcopy(curve)
	curve_js_blend = copy.deepcopy(curve_js)
	lam = np.cumsum(np.linalg.norm(np.diff(curve_blend,1,axis=0),2,1))
	lam = np.append(0,lam)
	zone_size = ldot/100*25 # zone size of fanuc is propotional to speed

	if lam[-1]/(len(breakpoints)-1) < zone_size:
		zone_size=lam[-1]/(len(breakpoints)-1)

	for i in range(1,len(breakpoints)-1):
		zone_start_id=breakpoints[i]-ceil(zone_size/(lam[breakpoints[i]]-lam[breakpoints[i]-1]))
		zone_end_id=breakpoints[i]+ceil(zone_size/(lam[breakpoints[i]+1]-lam[breakpoints[i]]))

		if zone_start_id < 1:
			zone_start_id=1
		if zone_end_id > len(lam)-2:
			zone_end_id=len(lam)-2

		blending=[]
		for j in range(3):
			xs=0
			xe=lam[zone_end_id]-lam[zone_start_id]
			A=[[xs**5,xs**4,xs**3,xs**2,xs,1],[5*xs**4,4*xs**3,3*xs**2,2*xs,1,0],[20*xs**3,12*xs**2,6*xs,2,0,0],\
				[xe**5,xe**4,xe**3,xe**2,xe,1],[5*xe**4,4*xe**3,3*xe**2,2*xe,1,0],[20*xe**3,12*xe**2,6*xe,2,0,0]]
			b=[curve_blend[zone_start_id,j],(curve_blend[zone_start_id,j]-curve_blend[zone_start_id-1,j])/(lam[zone_start_id]-lam[zone_start_id-1]),0,\
				curve_blend[zone_end_id,j],(curve_blend[zone_end_id+1,j]-curve_blend[zone_end_id,j])/(lam[zone_end_id+1]-lam[zone_end_id]),0]
			pp=np.matmul(np.linalg.pinv(A),b)
			blending.append(np.polyval(pp,lam[zone_start_id:zone_end_id+1]-lam[zone_start_id]))
		blending=np.array(blending)
		blending=blending.T
		curve_blend[zone_start_id:zone_end_id+1,:]=blending

		# for j in range(zone_start_id,zone_end_id+1):
		# 	curve_js_blend[j] = car2js(robot,curve_js_blend[j],curve_blend[j],curve_R[j])[0]

	return curve_js_blend,curve_blend,curve_R

def blend_js(q,breakpoints,lam,bp_exe_start_idx,bp_exe_end_idx):
	#q: 			full 50,000 joints
	#breakpoints:	breakpoints
	#lam: 			path length
	##return
	#q_blended: 	blended trajectory with movej

	q_blended=copy.deepcopy(q)
	skip=False
	merged_idx=[]

	for i in range(1,len(breakpoints)-1):
		if skip:
			merged_idx[-1].append(breakpoints[i])
			skip=False
			continue

		merged_idx.append([breakpoints[i]])

		start_idx	=bp_exe_start_idx
		end_idx	=bp_exe_end_idx

		if i+1<len(breakpoints)-1:
			if breakpoints[i+1]-breakpoints[i]<2*blending_num:
				skip=True
				start_idx	=breakpoints[i]-blending_num
				end_idx	=breakpoints[i+1]+blending_num

		for j in range(len(q[0])):
			poly = BPoly.from_derivatives([lam[start_idx],lam[end_idx]], [[q[start_idx,j],(q[start_idx,j]-q[start_idx-1,j])/(lam[start_idx]-lam[start_idx-1])], [q[end_idx,j],(q[end_idx+1,j]-q[end_idx,j])/(lam[end_idx+1]-lam[end_idx])]])
			q_blended[start_idx:end_idx,j]=poly(lam[start_idx:end_idx])


	return q_blended

def blend_exe():
	robot=abb6640(d=50)
	data_set='movel_30_car/'
	zone=100

	###maxium blending in RobotStudio, higher than that will result in similar behavior of z100
	z_max=min(zone,150)

	data = read_csv('data/'+data_set+'command.csv')
	breakpoints=np.array(data['breakpoints'].tolist())
	act_breakpoints=copy.deepcopy(breakpoints)
	act_breakpoints[1:]=act_breakpoints[1:]-1

	curve_js = read_csv('data/'+data_set+'Curve_js.csv',header=None).values
	curve = read_csv('data/'+data_set+'Curve_in_base_frame.csv',header=None).values




	data=read_csv('execution/'+data_set+'curve_exe_v500_z'+str(zone)+'.csv')
	q1=data[' J1'].tolist()
	q2=data[' J2'].tolist()
	q3=data[' J3'].tolist()
	q4=data[' J4'].tolist()
	q5=data[' J5'].tolist()
	q6=data[' J6'].tolist()
	timestamp=np.array(data['timestamp'].tolist()).astype(float)
	cmd_num=np.array(data[' cmd_num'].tolist()).astype(float)
	start_idx=np.where(cmd_num==5)[0][0]
	curve_exe_js=np.radians(np.vstack((q1,q2,q3,q4,q5,q6)).T.astype(float)[start_idx:])

	act_speed=[]
	lam_act=[0]
	curve_exe=[]
	for i in range(len(curve_exe_js)):
		robot_pose=robot.fwd(curve_exe_js[i])
		curve_exe.append(robot_pose.p)
		if i>0:
			lam_act.append(lam_act[-1]+np.linalg.norm(curve_exe[i]-curve_exe[i-1]))
		try:
			if timestamp[i-1]!=timestamp[i] and np.linalg.norm(curve_exe[-1]-curve_exe[-2])!=0:
				act_speed.append(np.linalg.norm(curve_exe[-1]-curve_exe[-2])/(timestamp[i]-timestamp[i-1]))
			else:
				act_speed.append(act_speed[-1])
				
		except IndexError:
			pass
	curve_exe=np.array(curve_exe)

	lam=calc_lam_js(curve_js,robot)
	lam_exe=calc_lam_cs(curve_exe)

	bp_exe=np.argmin(np.linalg.norm(curve_exe-curve[breakpoints[1]][:3],axis=1))

	# z_max+=30
	blending_start_idx=np.argmin(np.abs(lam-(lam[breakpoints[1]]-z_max)))
	blending_end_idx=np.argmin(np.abs(lam-(lam[breakpoints[1]]+z_max)))

	bp_exe_start_idx=np.argmin(np.linalg.norm(curve_exe-curve[blending_start_idx,:3],axis=1))
	bp_exe_end_idx=np.argmin(np.linalg.norm(curve_exe-curve[blending_end_idx,:3],axis=1))


	blending_points=[bp_exe_start_idx,bp_exe_end_idx]

	curve_blend_js=blend_js(curve_exe_js,[0,bp_exe,len(curve_exe_js)],lam_exe,bp_exe_start_idx,bp_exe_end_idx)

	curve_blend=[]
	for i in range(len(curve_blend_js)):
		curve_blend.append(robot.fwd(curve_blend_js[i]).p)
	curve_blend=np.array(curve_blend)


	###plot curves
	for i in range(6):
		plt.figure(i)
		plt.plot(lam,curve_js[:,i],label='original')
		plt.plot(lam_exe,curve_exe_js[:,i],label='execution')
		plt.plot(lam_exe,curve_blend_js[:,i],label='Blended J')
		plt.title('J'+str(i+1))
		plt.legend()

	###plot original curve
	plt.figure()
	ax = plt.axes(projection='3d')
	ax.plot3D(curve[:,0], curve[:,1],curve[:,2], 'red',label='original')
	ax.scatter3D(curve[act_breakpoints,0], curve[act_breakpoints,1],curve[act_breakpoints,2], 'blue')
	ax.scatter3D(curve_exe[blending_points,0], curve_exe[blending_points,1],curve_exe[blending_points,2], 'red')
	#plot execution curve
	ax.plot3D(curve_exe[:,0], curve_exe[:,1],curve_exe[:,2], 'green',label='execution')
	ax.plot3D(curve_blend[:,0], curve_blend[:,1],curve_blend[:,2], 'blue',label='blended')
	plt.legend()
	plt.title('Zone '+str(zone))
	plt.show()

def main():
	robot=abb6640(d=50)
	data_set='movel_30_ori/'

	data = read_csv('data/'+data_set+'command.csv')
	breakpoints=np.array(data['breakpoints'].tolist())
	act_breakpoints=copy.deepcopy(breakpoints)
	act_breakpoints[1:]=act_breakpoints[1:]-1

	curve_js = read_csv('data/'+data_set+'Curve_js.csv',header=None).values
	curve = read_csv('data/'+data_set+'Curve_in_base_frame.csv',header=None).values

	data=read_csv('execution/'+data_set+'curve_exe_v500_z10.csv')
	q1=data[' J1'].tolist()
	q2=data[' J2'].tolist()
	q3=data[' J3'].tolist()
	q4=data[' J4'].tolist()
	q5=data[' J5'].tolist()
	q6=data[' J6'].tolist()
	timestamp=np.array(data['timestamp'].tolist()).astype(float)
	cmd_num=np.array(data[' cmd_num'].tolist()).astype(float)
	start_idx=np.where(cmd_num==5)[0][0]
	curve_exe_js=np.radians(np.vstack((q1,q2,q3,q4,q5,q6)).T.astype(float)[start_idx:])

	act_speed=[]
	lam_act=[0]
	curve_exe=[]
	for i in range(len(curve_exe_js)):
		robot_pose=robot.fwd(curve_exe_js[i])
		curve_exe.append(robot_pose.p)
		if i>0:
			lam_act.append(lam_act[-1]+np.linalg.norm(curve_exe[i]-curve_exe[i-1]))
		try:
			if timestamp[i-1]!=timestamp[i] and np.linalg.norm(curve_exe[-1]-curve_exe[-2])!=0:
				act_speed.append(np.linalg.norm(curve_exe[-1]-curve_exe[-2])/(timestamp[i]-timestamp[i-1]))
			else:
				act_speed.append(act_speed[-1])
				
		except IndexError:
			pass
	curve_exe=np.array(curve_exe)

	
	lam=calc_lam_js(curve_js,robot)
	lam_exe=calc_lam_cs(curve_exe)


	curve_blend_js=blend_js(curve_js,breakpoints,lam,50)
	lamdot_blended=calc_lamdot(curve_blend_js,lam,robot,1)

	curve_blend=[]
	for i in range(len(curve_blend_js)):
		curve_blend.append(robot.fwd(curve_blend_js[i]).p)
	curve_blend=np.array(curve_blend)


	###plot curves
	for i in range(6):
		plt.figure(i)
		plt.plot(lam,curve_js[:,i],label='original')
		plt.plot(lam_exe,curve_exe_js[:,i],label='execution')
		plt.plot(lam,curve_blend_js[:,i],label='Blended J')
		plt.title('J'+str(i+1))
		plt.legend()

	###plot original curve
	plt.figure()
	ax = plt.axes(projection='3d')
	ax.plot3D(curve[:,0], curve[:,1],curve[:,2], 'red',label='original')
	ax.scatter3D(curve[act_breakpoints,0], curve[act_breakpoints,1],curve[act_breakpoints,2], 'blue')
	#plot execution curve
	ax.plot3D(curve_exe[:,0], curve_exe[:,1],curve_exe[:,2], 'green',label='execution')
	ax.plot3D(curve_blend[:,0], curve_blend[:,1],curve_blend[:,2], 'blue',label='blended')
	plt.show()


	##2d plot
	###plane projection visualization
	# A = np.array([curve[:,0], curve[:,1], np.ones(len(curve))]).T
	# b = curve[:,2]
	# c = np.linalg.lstsq(A,b,rcond=None)[0]

	# normal=np.array([c[0],c[1],-1])
	# normal=normal/np.linalg.norm(normal)

	# curve_2d = rodrigues_rot(curve[:,:3], normal, [0,0,1])
	# curve_exe_2d=rodrigues_rot(curve_exe, normal, [0,0,1])
	# curve_blend_2d=rodrigues_rot(curve_blend, normal, [0,0,1])

	# plt.figure()
	# ax = plt.axes(projection='3d')
	# ax.plot3D(curve_2d[:,0], curve_2d[:,1],curve_2d[:,2], 'red',label='original')
	# ax.scatter3D(curve_2d[act_breakpoints,0], curve_2d[act_breakpoints,1],curve_2d[act_breakpoints,2], 'blue')
	# #plot execution curve
	# ax.plot3D(curve_exe_2d[:,0], curve_exe_2d[:,1],curve_exe_2d[:,2], 'green',label='execution')
	# #plot arb blended
	# ax.plot3D(curve_blend_2d[:,0], curve_blend_2d[:,1],curve_blend_2d[:,2], 'blue',label='blended')
	# plt.legend()
	# plt.show()

def main2():
	robot=abb6640(d=50)
	data_set='movel_30_car/'

	data = read_csv('data/'+data_set+'command.csv')
	breakpoints=np.array(data['breakpoints'].tolist())
	primitives=data['primitives'].tolist()[1:]
	act_breakpoints=copy.deepcopy(breakpoints)
	act_breakpoints[1:]=act_breakpoints[1:]-1

	curve_js = read_csv('data/'+data_set+'Curve_js.csv',header=None).values
	curve = read_csv('data/'+data_set+'Curve_in_base_frame.csv',header=None).values




	data=read_csv('execution/'+data_set+'curve_exe_v500_z10.csv')
	q1=data[' J1'].tolist()
	q2=data[' J2'].tolist()
	q3=data[' J3'].tolist()
	q4=data[' J4'].tolist()
	q5=data[' J5'].tolist()
	q6=data[' J6'].tolist()
	timestamp=np.array(data['timestamp'].tolist()).astype(float)
	cmd_num=np.array(data[' cmd_num'].tolist()).astype(float)
	start_idx=np.where(cmd_num==5)[0][0]
	curve_exe_js=np.radians(np.vstack((q1,q2,q3,q4,q5,q6)).T.astype(float)[start_idx:])

	act_speed=[]
	lam_act=[0]
	curve_exe=[]
	for i in range(len(curve_exe_js)):
		robot_pose=robot.fwd(curve_exe_js[i])
		curve_exe.append(robot_pose.p)
		if i>0:
			lam_act.append(lam_act[-1]+np.linalg.norm(curve_exe[i]-curve_exe[i-1]))
		try:
			if timestamp[i-1]!=timestamp[i] and np.linalg.norm(curve_exe[-1]-curve_exe[-2])!=0:
				act_speed.append(np.linalg.norm(curve_exe[-1]-curve_exe[-2])/(timestamp[i]-timestamp[i-1]))
			else:
				act_speed.append(act_speed[-1])
				
		except IndexError:
			pass
	curve_exe=np.array(curve_exe)

	
	lam=calc_lam_js(curve_js,robot)


	act_breakpoints=breakpoints
	act_breakpoints[1:]=act_breakpoints[1:]-1
	lam_blended,q_blended=blend_js_from_primitive(curve_js[act_breakpoints],curve,breakpoints,lam,primitives,robot)

	curve_blend=[]
	for i in range(len(q_blended)):
		curve_blend.append(robot.fwd(q_blended[i]).p)
	curve_blend=np.array(curve_blend)


	###plot curves
	plt.figure()
	# plt.title(s+' '+z)
	ax = plt.axes(projection='3d')
	ax.plot3D(curve[:,0], curve[:,1],curve[:,2], 'red',label='original')
	ax.scatter3D(curve[act_breakpoints,0], curve[act_breakpoints,1],curve[act_breakpoints,2], 'blue')
	#plot execution curve
	ax.plot3D(curve_exe[:,0], curve_exe[:,1],curve_exe[:,2], 'green',label='execution')
	#plot arb blended
	ax.plot3D(curve_blend[:,0], curve_blend[:,1],curve_blend[:,2], 'blue',label='original')
	plt.show()

def test_blending_with_primitives():

	data_dir='../simulation/robotstudio_sim/scripts/fitting_output_new/all_theta_opt_blended/'

	data = read_csv(data_dir+'command.csv')
	breakpoints=np.array(data['breakpoints'].tolist())
	primitives=data['primitives'].tolist()[1:]

	col_names=['J1', 'J2','J3', 'J4', 'J5', 'J6'] 
	data=read_csv(data_dir+'all_theta_opt_js.csv',names=col_names)
	q1=data['J1'].tolist()
	q2=data['J2'].tolist()
	q3=data['J3'].tolist()
	q4=data['J4'].tolist()
	q5=data['J5'].tolist()
	q6=data['J6'].tolist()
	curve_js=np.vstack((q1,q2,q3,q4,q5,q6)).T.astype(float)

	data = read_csv(data_dir+'curve_fit.csv')
	curve_x=data['x'].tolist()
	curve_y=data['y'].tolist()
	curve_z=data['z'].tolist()
	curve=np.vstack((curve_x, curve_y, curve_z)).T

	col_names=['timestamp', 'cmd_num', 'J1', 'J2','J3', 'J4', 'J5', 'J6'] 
	data=read_csv(data_dir+'curve_exe_vmax_z10.csv',names=col_names)
	q1=data['J1'].tolist()[1:]
	q2=data['J2'].tolist()[1:]
	q3=data['J3'].tolist()[1:]
	q4=data['J4'].tolist()[1:]
	q5=data['J5'].tolist()[1:]
	q6=data['J6'].tolist()[1:]
	timestamp=np.array(data['timestamp'].tolist()[1:]).astype(float)
	cmd_num=np.array(data['cmd_num'].tolist()[1:]).astype(float)
	start_idx=np.where(cmd_num==5)[0][0]
	curve_exe_js=np.radians(np.vstack((q1,q2,q3,q4,q5,q6)).T.astype(float)[start_idx:])
	timestep=np.average(timestamp[1:]-timestamp[:-1])

	robot=abb6640(d=50)

	act_speed=[]
	lam_act=[0]
	curve_exe=[]
	for i in range(len(curve_exe_js)):
		robot_pose=robot.fwd(curve_exe_js[i])
		curve_exe.append(robot_pose.p)
		if i>0:
			lam_act.append(lam_act[-1]+np.linalg.norm(curve_exe[i]-curve_exe[i-1]))
		try:
			if timestamp[-1]!=timestamp[-2]:
				act_speed.append(np.linalg.norm(curve_exe[-1]-curve_exe[-2])/timestep)
				
		except IndexError:
			pass



	lam=calc_lam_js(curve_js,robot)
	lam_act=calc_lam_js(curve_exe_js,robot)

	lamdot_fit=calc_lamdot(curve_js,lam,robot,1)

	act_breakpoints=breakpoints
	act_breakpoints[1:]=act_breakpoints[1:]-1
	lam_blended,q_blended=blend_js_from_primitive(curve_js[act_breakpoints],curve,breakpoints,lam,primitives,robot)
	lamdot_blended=calc_lamdot(q_blended,lam_blended,robot,1)
	lamdot_act=calc_lamdot(curve_exe_js,lam_act,robot,1)


	plt.plot(lam,lamdot_fit, label='Fitting')
	plt.plot(lam_blended,lamdot_blended, label='Blended')
	plt.plot(lam_act[1:],act_speed, label='Actual Speed')
	# plt.ylim(0,2100)
	plt.title("speed vs lambda")
	plt.ylabel('speed (mm/s)')
	plt.xlabel('lambda (mm)')
	plt.legend()
	plt.show()

if __name__ == '__main__':
	blend_exe()
	# main()

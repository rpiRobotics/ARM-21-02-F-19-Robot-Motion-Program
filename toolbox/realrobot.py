from io import StringIO
from pandas import read_csv
from sklearn.cluster import KMeans
import numpy as np
import scipy
from utils import *
import time

def average_curve(curve_all,timestamp_all):
	###get desired synced timestamp first
	max_length=[]
	max_time=[]
	for i in range(len(timestamp_all)):
		max_length.append(len(timestamp_all[i]))
		max_time.append(timestamp_all[i][-1])
	max_length=np.max(max_length)
	max_time=np.max(max_time)
	timestamp_d=np.linspace(0,max_time,num=max_length)

	###linear interpolate each curve with synced timestamp
	curve_all_new=[]
	for i in range(len(timestamp_all)):
		curve_all_new.append(interplate_timestamp(curve_all[i],timestamp_all[i],timestamp_d))

	curve_all_new=np.array(curve_all_new)

	return curve_all_new, np.average(curve_all_new,axis=0),timestamp_d

def remove_traj_outlier(curve_exe_js_all,timestamp_all,total_time_all):

	km = KMeans(n_clusters=2)
	index=km.fit_predict(np.array(total_time_all).reshape(-1,1))
	cluster=km.cluster_centers_
	major_index=scipy.stats.mode(index)[0][0]       ###mostly appeared index
	major_indices=np.where(index==major_index)[0]
	time_mode_avg=cluster[major_index]

	if abs(cluster[0][0]-cluster[1][0])>0.02*time_mode_avg:
		curve_exe_js_all=[curve_exe_js_all[iii] for iii in major_indices]
		timestamp_all=[timestamp_all[iii] for iii in major_indices]
		print('outlier traj detected')

	return curve_exe_js_all,timestamp_all

def average_N_exe(ms,robot,primitives,breakpoints,p_bp,q_bp,v,z,curve,log_path='',N=5):
	###N run execute
	curve_exe_js_all=[]
	timestamp_all=[]
	total_time_all=[]

	for r in range(N):
		log_results=ms.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,v,z)
		###save 5 runs
		if len(log_path)>0:
			# Write log csv to file
			np.savetxt(log_path+'/run_'+str(r)+'.csv',log_results.data,delimiter=',',comments='',header='timestamp,cmd_num,J1,J2,J3,J4,J5,J6')

		##############################data analysis#####################################
		lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.logged_data_analysis(robot,log_results,realrobot=True)

		###throw bad curves
		_, _, _,_, _, timestamp_temp=ms.chop_extension(curve_exe, curve_exe_R,curve_exe_js, speed, timestamp,curve[0,:3],curve[-1,:3])
		total_time_all.append(timestamp_temp[-1]-timestamp_temp[0])

		timestamp=timestamp-timestamp[0]

		curve_exe_js_all.append(curve_exe_js)
		timestamp_all.append(timestamp)
		time.sleep(0.5)
	###trajectory outlier detection, based on chopped time
	curve_exe_js_all,timestamp_all=remove_traj_outlier(curve_exe_js_all,timestamp_all,total_time_all)

	###infer average curve from linear interplateion
	curve_js_all_new, avg_curve_js, timestamp_d=average_curve(curve_exe_js_all,timestamp_all)

	return curve_js_all_new, avg_curve_js, timestamp_d

def average_N_exe_multimove(ms,breakpoints,robot1,primitives1,p_bp1,q_bp1,v1_all,z1_all,robot2,primitives2,p_bp2,q_bp2,v2_all,z2_all,relative_path,safeq1=None,safeq2=None,log_path='',N=5):
	###N run execute
	curve_exe_js_all=[]
	timestamp_all=[]
	total_time_all=[]

	for r in range(N):
		if safeq1:
			ms.jog_joint_multimove(safeq1,safeq2)

		log_results=ms.exec_motions_multimove(robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,v1_all,v2_all,z1_all,z2_all)
		###save 5 runs
		if len(log_path)>0:
			# Write log csv to file
			np.savetxt(log_path+'/run_'+str(r)+'.csv',log_results.data,delimiter=',',comments='',header='timestamp,cmd_num,J1,J2,J3,J4,J5,J6,J1_2,J2_2,J3_2,J4_2,J5_2,J6_2')

		##############################data analysis#####################################
		lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe, relative_path_exe_R=ms.logged_data_analysis_multimove(log_results,robot1,robot2,realrobot=True)

		curve_exe_js_dual=np.hstack((curve_exe_js1,curve_exe_js2))
		###throw bad curves
		_, _,_,_,_,_,_, _, timestamp_temp, _, _=\
			ms.chop_extension_dual(lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R,relative_path[0,:3],relative_path[-1,:3])
		total_time_all.append(timestamp_temp[-1]-timestamp_temp[0])

		timestamp=timestamp-timestamp[0]

		curve_exe_js_all.append(curve_exe_js_dual)
		timestamp_all.append(timestamp)
		time.sleep(0.5)

	###trajectory outlier detection, based on chopped time
	curve_exe_js_all,timestamp_all=remove_traj_outlier(curve_exe_js_all,timestamp_all,total_time_all)

	###infer average curve from linear interplateion
	curve_js_all_new, avg_curve_js, timestamp_d=average_curve(curve_exe_js_all,timestamp_all)

	return curve_js_all_new, avg_curve_js, timestamp_d


def average_5_egm_car_exe(et,curve_cmd,curve_cmd_R):
	###5 run execute egm Cartesian
	curve_exe_js_all=[]
	timestamp_all=[]
	for r in range(5):
		###move to start first
		print('moving to start point')
		et.jog_joint_cartesian(curve_cmd[0],curve_cmd_R[0])
		
		###traverse the curve
		timestamp,curve_exe_js=et.traverse_curve_cartesian(curve_cmd,curve_cmd_R)

		timestamp=timestamp-timestamp[0]
		curve_exe_js_all.append(curve_exe_js)
		timestamp_all.append(timestamp)
		time.sleep(0.5)

	###infer average curve from linear interplateion
	curve_js_all_new, avg_curve_js, timestamp_d=average_curve(curve_exe_js_all,timestamp_all)

	return curve_js_all_new,avg_curve_js, timestamp_d

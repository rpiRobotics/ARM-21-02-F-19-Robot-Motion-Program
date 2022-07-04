from general_robotics_toolbox import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy import signal
import scipy
from sklearn.cluster import KMeans

def remove_traj_outlier(curve_exe_all,curve_exe_js_all,timestamp_all,total_time_all):

	km = KMeans(n_clusters=2)
	index=km.fit_predict(np.array(total_time_all).reshape(-1,1))
	cluster=km.cluster_centers_
	major_index=scipy.stats.mode(index)[0][0]       ###mostly appeared index
	major_indices=np.where(index==major_index)[0]
	time_mode_avg=cluster[major_index]

	if abs(cluster[0][0]-cluster[1][0])>0.02*time_mode_avg:
		curve_exe_all=[curve_exe_all[iii] for iii in major_indices]
		curve_exe_js_all=[curve_exe_js_all[iii] for iii in major_indices]
		timestamp_all=[timestamp_all[iii] for iii in major_indices]
		print('outlier traj detected')

	return curve_exe_all,curve_exe_js_all,timestamp_all

def interplate_timestamp(curve,timestamp,timestamp_d):

	curve_new=[]
	for i in range(len(curve[0])):
		# curve_new.append(np.interp(timestamp_d,timestamp,curve[:,i]))
		curve_new.append(scipy.interpolate.CubicSpline(timestamp, curve[:,i])(timestamp_d))

	return np.array(curve_new).T

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
	
def replace_outliers2(data):
	rolling_window=30
	for i in range(len(data)-rolling_window):
		rolling_avg=np.mean(data[i:i+rolling_window])
		if np.abs(data[i]-rolling_avg)>0.0001*rolling_avg:
			data[i]=rolling_avg
	return data
def replace_outliers(data, m=2):
	data[abs(data - np.mean(data)) > m * np.std(data)] = np.mean(data)
	return data

def quadrant(q):
	temp=np.ceil(np.array([q[0],q[3],q[5]])/(np.pi/2))-1
	
	if q[4] < 0:
		last = 1
	else:
		last = 0

	return np.hstack((temp,[last])).astype(int)
	
def cross(v):
	return np.array([[0,-v[-1],v[1]],
					[v[-1],0,-v[0]],
					[-v[1],v[0],0]])

def direction2R(v_norm,v_tang):
	v_norm=v_norm/np.linalg.norm(v_norm)
	v_tang=VectorPlaneProjection(v_tang,v_norm)
	y=np.cross(v_norm,v_tang)

	R=np.vstack((v_tang,y,v_norm)).T

	return R

def LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
 
	ndotu = planeNormal.dot(rayDirection)
	if abs(ndotu) < epsilon:
		raise RuntimeError("no intersection or line is within plane")
 
	w = rayPoint - planePoint
	si = -planeNormal.dot(w) / ndotu
	Psi = w + si * rayDirection + planePoint
	return Psi

def VectorPlaneProjection(v,n):
	temp = (np.dot(v, n)/np.linalg.norm(n)**2)*n
	v_out=v-temp
	v_out=v_out/np.linalg.norm(v_out)
	return v_out

def find_j_min(robot,curve_js):
	sing_min=[]
	for q in curve_js:
		u, s, vh = np.linalg.svd(robot.jacobian(q))
		sing_min.append(s[-1])

	return sing_min

def get_angle(v1,v2,less90=False):
	v1=v1/np.linalg.norm(v1)
	v2=v2/np.linalg.norm(v2)
	dot=np.dot(v1,v2)
	if dot>0.99999999999:
		return 0
	elif dot<-0.99999999999:
		return np.pi
	angle=np.arccos(dot)
	if less90 and angle>np.pi/2:
		angle=np.pi-angle
	return angle


def lineFromPoints(P, Q):
	#return coeff ax+by+c=0
	a = Q[1] - P[1]
	b = P[0] - Q[0]
	c = -(a*(P[0]) + b*(P[1]))
	return a,b,c

def extract_points(primitive_type,points):
	if primitive_type=='movec_fit':
		endpoints=points[8:-3].split('array')
		endpoint1=endpoints[0][:-4].split(',')
		endpoint2=endpoints[1][2:].split(',')

		return np.array(list(map(float, endpoint1))),np.array(list(map(float, endpoint2)))
	else:
		endpoint=points[8:-3].split(',')
		return np.array(list(map(float, endpoint)))


def visualize_curve_w_normal(curve,curve_normal,stepsize=500,equal_axis=False):
	curve=curve[::stepsize]
	curve_normal=curve_normal[::stepsize]
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot3D(curve[:,0], curve[:,1],curve[:,2], 'gray')
	ax.quiver(curve[:,0],curve[:,1],curve[:,2],30*curve_normal[:,0],30*curve_normal[:,1],30*curve_normal[:,2])
	ax.set_xlabel('x (mm)')
	ax.set_ylabel('y (mm)')
	ax.set_zlabel('z (mm)')
	if equal_axis:
		ax.set_xlim([0,3000])
		ax.set_ylim([0,3000])
		ax.set_zlim([0,3000])

	plt.show()

def visualize_curve(curve,stepsize=10):
	curve=curve[::stepsize]
	plt.figure()
	ax = plt.axes(projection='3d')
	ax.plot3D(curve[:,0], curve[:,1],curve[:,2], 'gray')

	plt.show()

def linear_interp(x,y):
	f=interp1d(x,y.T)
	x_new=np.linspace(x[0],x[-1],len(x))
	return x_new, f(x_new).T

def moving_average(a, n=10) :
	ret = np.cumsum(a, axis=0)
	ret[n:] = ret[n:] - ret[:-n]
	return ret[n - 1:] / n


def lfilter(x, y):
	x,y=linear_interp(x,y)
	n=10
	y1=moving_average(y,n)
	y2=moving_average(np.flip(y,axis=0),n)
	return x[int(n/2):-int(n/2)+1], (y1+np.flip(y2,axis=0))/2

def orientation_interp(R_init,R_end,steps):
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

def H_from_RT(R,T):
	return np.hstack((np.vstack((R,np.zeros(3))),np.append(T,1).reshape(4,1)))


def car2js(robot,q_init,curve_fit,curve_fit_R):
	###calculate corresponding joint configs
	curve_fit_js=[]
	if curve_fit.shape==(3,):
		q_all=np.array(robot.inv(curve_fit,curve_fit_R))

		###choose inv_kin closest to previous joints
		if len(curve_fit_js)>1:
			temp_q=q_all-curve_fit_js[-1]
		else:
			temp_q=q_all-q_init
		order=np.argsort(np.linalg.norm(temp_q,axis=1))
		curve_fit_js.append(q_all[order[0]])
	else:
		for i in range(len(curve_fit)):
			q_all=np.array(robot.inv(curve_fit[i],curve_fit_R[i]))

			###choose inv_kin closest to previous joints
			if len(curve_fit_js)>1:
				temp_q=q_all-curve_fit_js[-1]
			else:
				temp_q=q_all-q_init
			order=np.argsort(np.linalg.norm(temp_q,axis=1))
			curve_fit_js.append(q_all[order[0]])
	return curve_fit_js

def R2w(curve_R,R_constraint=[]):
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
def w2R(curve_w,R_init):
	curve_R=[]
	for i in range(len(curve_w)):
		theta=np.linalg.norm(curve_w[i])
		if theta==0:
			curve_R.append(R_init)
		else:
			curve_R.append(np.dot(rot(curve_w[i]/theta,theta),R_init))

	return np.array(curve_R)

def unwrapped_angle_check(q_init,q_all):

    temp_q=q_all-q_init
    temp_q = np.unwrap(temp_q)
    order=np.argsort(np.linalg.norm(temp_q,axis=1))
    # return q_all[order[0]]
    return temp_q[order[0]]+q_init
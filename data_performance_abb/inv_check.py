import numpy as np
from pandas import *
import sys
from general_robotics_toolbox import *

sys.path.append('../toolbox')
from robot_def import *
from utils import *


col_names=['X', 'Y', 'Z','direction_x','direction_y','direction_z'] 
data = read_csv("Curve_interp.csv", names=col_names)
curve_x=data['X'].tolist()
curve_y=data['Y'].tolist()
curve_z=data['Z'].tolist()
curve_direction_x=data['direction_x'].tolist()
curve_direction_y=data['direction_y'].tolist()
curve_direction_z=data['direction_z'].tolist()

###read interpolated curves in joint space
col_names=['q1', 'q2', 'q3','q4', 'q5', 'q6'] 
data = read_csv("Curve_js.csv", names=col_names)
curve_q1=data['q1'].tolist()
curve_q2=data['q2'].tolist()
curve_q3=data['q3'].tolist()
curve_q4=data['q4'].tolist()
curve_q5=data['q5'].tolist()
curve_q6=data['q6'].tolist()
curve_js=np.vstack((curve_q1, curve_q2, curve_q3,curve_q4,curve_q5,curve_q6)).T

curve=np.vstack((curve_x, curve_y, curve_z)).T
curve_direction=np.vstack((curve_direction_x, curve_direction_y, curve_direction_z)).T

curve_base=np.zeros(curve.shape)
curve_R_base=[]


###reference frame transformation
R=np.array([[0,0,1.],
			[1.,0,0],
			[0,1.,0]])
T=np.array([[2700.],[-800.],[500.]])
H=np.vstack((np.hstack((R,T)),np.array([0,0,0,1])))

###checkpoint1
# print(np.dot(R,direction2R(curve_direction[0],curve[1]-curve[0])))
for i in range(len(curve)):
	curve_base[i]=np.dot(H,np.hstack((curve[i],[1])).T)[:-1]
	try:
		R_curve=direction2R(curve_direction[i],curve[i+1]-curve[i])

	except:
		pass
	curve_R_base.append(np.dot(R,R_curve))
	


###units
curve_base=curve_base/1000.

# curve_js=np.zeros((len(curve),6))
# q_init=np.radians([35.406892, 12.788519, 27.907507, -89.251430, 52.417435, -128.363215])

# for i in range(len(curve_base)):
# 	q_all=inv(curve_base[i],curve_R_base[i])
# 	###choose inv_kin closest to previous joints
# 	if i==0:
# 		temp_q=q_all-q_init
# 		order=np.argsort(np.linalg.norm(temp_q,axis=1))
# 		curve_js[i]=q_all[order[0]]
# 	else:
# 		temp_q=q_all-curve_js[i-1]
# 		order=np.argsort(np.linalg.norm(temp_q,axis=1))
# 		curve_js[i]=q_all[order[0]]


###checkpoint3	#check invkin
H=np.vstack((np.hstack((R.T,-np.dot(R.T,T))),np.array([0,0,0,1])))
curve_base_temp=np.zeros(curve.shape)
error_in_base=np.zeros(curve.shape)
for i in range(len(curve_js)):
	curve_base_temp[i]=(np.dot(H,np.hstack((1000.*fwd(curve_js[i]).p,[1])).T)[:-1])
	error_in_base[i]=1000.*fwd(curve_js[i]).p-1000.*curve_base[i]
print(np.max(np.linalg.norm(error_in_base,axis=1)))

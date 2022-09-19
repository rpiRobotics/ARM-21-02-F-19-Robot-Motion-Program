from math import radians,pi
import numpy as np
from pandas import *
import yaml
from matplotlib import pyplot as plt

from general_robotics_toolbox import *
from general_robotics_toolbox.general_robotics_toolbox_invkin import *
import sys


# from simulation.roboguide.fanuc_toolbox.fanuc_client import FANUCClient, TPMotionProgram, joint2robtarget, jointtarget, robtarget
# from toolbox.robots_def import arb_robot, m900ia
sys.path.append('toolbox')
sys.path.append('../toolbox')
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
    # R=np.vstack((Vy,np.cross(-N,Vy),-N))
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
        try:
            R_curve=direction2R(curve_normal[i],-curve[i+1]+curve[i])
        except:
            traceback.print_exc()
            pass	
        curve_R.append(R_curve)

    ###insert initial orientation
    curve_R.insert(-1,curve_R[-1])

    ###get all possible initial config
    try:
        # q_inits=np.array(robot.inv(curve[0],curve_R[0]))
        q_inits
    except:
        print('no solution available')
        return []

    curve_js_all=[]
    for q_init in q_inits:

        curve_js=np.zeros((len(curve),6))
        curve_js[0]=q_init
        for i in range(1,len(curve)):
            q_all=np.array(robot.inv(curve[i],curve_R[i]))
            if len(q_all)==0:
                return

            # temp_q=q_all-curve_js[i-1]
            # order=np.argsort(np.linalg.norm(temp_q,axis=1))
            temp_q = unwrapped_angle_check(curve_js[i-1],q_all)
            if np.linalg.norm(temp_q-curve_js[i-1])>0.5:
                break	#if large changes in q
            else:
                curve_js[i]=temp_q

        #check if all q found
        if np.linalg.norm(curve_js[-1])>0:
            curve_js_all.append(curve_js)

    return curve_js_all

def main():

    obj_type='wood'
    # obj_type='blade'
    data_dir='../data/baseline_m710ic/'+obj_type+'/'
    print(obj_type)

    # define robot
    robot=m710ic(d=50)
    # robot=m900ia(d=50)
    # robot_fake=abb6640(d=50)

    # the original curve in Cartesian space
    if obj_type=='wood':
        curve = read_csv("../../../data/wood/Curve_dense.csv",header=None).values
    else:
        # curve = read_csv("../../../data/from_NX/Curve_in_base_frame.csv",header=None).values
        curve = read_csv("../../../data/from_NX/Curve_dense.csv",header=None).values
    lam=calc_lam_cs(curve)

    # put the curve in the best pose relative to the robot
    H = pose_opt(robot,curve[:,:3],curve[:,3:])
    print(H)
    curve_base,curve_normal_base=curve_frame_conversion(curve[:,:3],curve[:,3:],H)

    curve_js_all=find_js(robot,curve_base,curve_normal_base)

    print(len(curve_js_all))
    # exit()

    J_min=[]
    for i in range(len(curve_js_all)):
        J_min.append(find_j_min(robot,curve_js_all[i]))

    J_min=np.array(J_min)
    # plt.figure()
    # for i in range(len(J_min)):
    #     plt.plot(lam,J_min[i],label="inv choice "+str(i))

    # plt.legend()
    # plt.title('Minimum J_SINGULAR')
    # plt.show()
    curve_js=curve_js_all[np.argmin(J_min.min(axis=1))]

    ###save file
    df=DataFrame({'x':curve_base[:,0],'y':curve_base[:,1], 'z':curve_base[:,2],'x_dir':curve_normal_base[:,0],'y_dir':curve_normal_base[:,1], 'z_dir':curve_normal_base[:,2]})
    df.to_csv(data_dir+'Curve_in_base_frame.csv',header=False,index=False)
    DataFrame(curve_js).to_csv(data_dir+'Curve_js.csv',header=False,index=False)
    with open(data_dir+'blade_pose.yaml', 'w') as file:
        documents = yaml.dump({'H':H.tolist()}, file)

if __name__ == '__main__':
    main()
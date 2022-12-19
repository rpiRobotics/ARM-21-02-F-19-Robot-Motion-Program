import numpy as np
from general_robotics_toolbox import *
from pandas import read_csv
import sys

sys.path.append('../../../toolbox')
from robots_def import *
from error_check import *
from MotionSend import *
from lambda_calc import *
from realrobot import *

def main():
    realrobot=True
    v_start=500
    dataset='curve_2/'
    solution_dir='curve_pose_opt2/'
    data_dir=dataset+solution_dir
    cmd_dir=data_dir+'100L/'


    robot=robot_obj('ABB_6640_180_255','../config/abb_6640_180_255_robot_default_config.yml',tool_file_path='../config/paintgun.csv',d=50,acc_dict_path='')
    curve = read_csv(data_dir+"Curve_in_base_frame.csv",header=None).values

    if realrobot:
        ms = MotionSend(url='http://192.168.55.1:80')
    else:
        ms = MotionSend()

    breakpoints,primitives, p_bp,q_bp=ms.extract_data_from_cmd(cmd_dir+"command.csv")
    p_bp, q_bp = ms.extend(robot, q_bp, primitives, breakpoints, p_bp,extension_start=100,extension_end=100)

    v=v_start
    v_prev=v_start*2
    v_prev_possible=v_start/2
    
    max_error_threshold=0.5
    max_ori_threshold=np.radians(3)
    max_error=999
    while True:

        ###update velocity profile
        v_cmd = speeddata(v,9999999,9999999,999999)
        #execute 
        if realrobot:
            curve_js_all_new, curve_exe_js, timestamp=average_N_exe(ms,robot,primitives,breakpoints,p_bp,q_bp,v_cmd,z50,curve,log_path=cmd_dir+'realrobot',N=5)
            ###calculat data with average curve
            lam, curve_exe, curve_exe_R, speed=logged_data_analysis(robot,timestamp,curve_exe_js)
        else:
            log_results = ms.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,v_cmd,z100)
            ##############################data analysis#####################################
            lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.logged_data_analysis(robot,log_results,realrobot=True)
        #############################chop extension off##################################
        lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.chop_extension(curve_exe, curve_exe_R,curve_exe_js, speed, timestamp,curve[0,:3],curve[-1,:3])
        ##############################calcualte error########################################
        error,angle_error=calc_all_error_w_normal(curve_exe,curve[:,:3],curve_exe_R[:,:,-1],curve[:,3:])
        
        max_error=max(error)
        max_angle_error=max(angle_error)
        print(v,max_error)
        v_prev_temp=v
        if max_error>max_error_threshold or max_angle_error>max_ori_threshold:
            v-=abs(v_prev-v)/2
        else:
            v_prev_possible=v
            #stop condition
            if max_error_threshold-max_error<max_error_threshold/20 or max_ori_threshold-max_angle_error<max_ori_threshold/20:
                break   
            v+=abs(v_prev-v)/2

        v_prev=v_prev_temp

        #if stuck
        if abs(v-v_prev)<1:
            v=v_prev_possible
            v_cmd = speeddata(v,9999999,9999999,999999)

            log_results = ms.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,v_cmd,z100)
            ##############################data analysis#####################################
            lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.logged_data_analysis(robot,log_results,realrobot=True)
            #############################chop extension off##################################
            lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.chop_extension(curve_exe, curve_exe_R,curve_exe_js, speed, timestamp,curve[0,:3],curve[-1,:3])
            ##############################calcualte error########################################
            error,angle_error=calc_all_error_w_normal(curve_exe,curve[:,:3],curve_exe_R[:,:,-1],curve[:,3:])
            
            max_error=max(error)
            break

    


 
    ######################################save <1mm logged train_data##############################################
    df=DataFrame({'average speed':[np.average(speed)],'max speed':[np.amax(speed)],'min speed':[np.amin(speed)],'std speed':[np.std(speed)],\
        'average error':[np.average(error)],'max error':[max_error],'min error':[np.amin(error)],'std error':[np.std(error)],\
        'average angle error':[np.average(angle_error)],'max angle error':[max(angle_error)],'min angle error':[np.amin(angle_error)],'std angle error':[np.std(angle_error)]})

    
    if realrobot:
        df.to_csv(cmd_dir+'realrobot/speed_info.csv',header=True,index=False)
    else:
        np.savetxt(cmd_dir+"curve_exe"+"_v"+str(v)+"_z10.csv",log_results.data,delimiter=',',header='timestamp,cmd_num,J1,J2,J3,J4,J5,J6')
        df.to_csv(cmd_dir+'speed_info.csv',header=True,index=False)

if __name__ == "__main__":
    main()
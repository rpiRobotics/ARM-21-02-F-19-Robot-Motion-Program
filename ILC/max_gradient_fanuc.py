import numpy as np
from scipy.signal import find_peaks
from general_robotics_toolbox import *
from pandas import read_csv,DataFrame
import sys
from io import StringIO
from matplotlib import pyplot as plt
from pathlib import Path
import os
import argparse
from fanuc_motion_program_exec_client import *

sys.path.append('ILC')
from ilc_toolbox import *
sys.path.append('../toolbox')
sys.path.append('toolbox')
from robots_def import *
from error_check import *
from lambda_calc import *
from blending import *
from fanuc_utils import *

def max_grad_descent(filepath,robot,velocity,desired_curve,desired_curve_js,\
    error_tol=0.5,angerror_tol=3,velstd_tol=5,iteration_max=100):

    curve=desired_curve
    curve_js=desired_curve_js

    ilc_output=filepath+'/result_speed_'+str(velocity)+'/'
    Path(ilc_output).mkdir(exist_ok=True)

    try:
        breakpoints,primitives,p_bp,q_bp=extract_data_from_cmd(filepath+'/command.csv')
    except:
        print("Ecountering Error while reading cmd file.")
        print("Convert desired curve to command")

        total_seg = 100
        step=int((len(curve_js)-1)/total_seg)
        breakpoints = [0]
        primitives = ['movej_fit']
        q_bp = [[curve_js[0]]]
        p_bp = [[robot.fwd(curve_js[0]).p]]
        for i in range(step,len(curve_js),step):
            breakpoints.append(i)
            primitives.append('movel_fit')
            q_bp.append([curve_js[i]])
            p_bp.append([robot.fwd(curve_js[i]).p])
        
        # df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'points':p_bp,'q_bp':q_bp})
        # df.to_csv(filepath+'/command.csv',header=True,index=False)

    ### speed,zone
    s = velocity
    z = 100 # zone (corner path), CNT100

    ### Gradient descent parameters
    multi_peak_threshold=0.2 # decreasing peak higher than threshold
    alpha = 0.5 # gradient descentz step size
    
    primitives,p_bp,q_bp=extend_start_end(robot,q_bp,primitives,breakpoints,p_bp,extension_d=60)

    ###ilc toolbox def
    ilc=ilc_toolbox(robot,primitives)

    ###TODO: extension fix start point, moveC support
    ms = MotionSendFANUC()
    draw_error_max=None
    draw_speed_max=None
    for i in range(iteration_max):
        
        ###execute,curve_fit_js only used for orientation
        logged_data=ms.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,s,z)

        # print(logged_data)
        StringData=StringIO(logged_data.decode('utf-8'))
        df = read_csv(StringData)
        ##############################data analysis#####################################
        lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.logged_data_analysis(robot,df)

        #############################chop extension off##################################
        lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.chop_extension(curve_exe, curve_exe_R,curve_exe_js, speed, timestamp,curve[:,:3],curve[:,3:])
        ave_speed=np.mean(speed)
        std_speed=np.std(speed)

        ##############################calcualte error########################################
        error,angle_error=calc_all_error_w_normal(curve_exe,curve[:,:3],curve_exe_R[:,:,-1],curve[:,3:])
        print('Iteration:',i,', Max Error:',max(error),', Ave. Speed:',ave_speed, ', Speed Std:',std_speed, 'Speed Std/Ave:',(ave_speed/std_speed*100),'%')
        #############################error peak detection###############################
        peaks,_=find_peaks(error,height=multi_peak_threshold,prominence=0.05,distance=20/(lam[int(len(lam)/2)]-lam[int(len(lam)/2)-1]))		###only push down peaks higher than height, distance between each peak is 20mm, threshold to filter noisy peaks

        if len(peaks)==0 or np.argmax(error) not in peaks:
            peaks=np.append(peaks,np.argmax(error))
        
        ##############################plot error#####################################
        fig, ax1 = plt.subplots()
        ax2 = ax1.twinx()
        ax1.plot(lam, speed, 'g-', label='Speed')
        ax2.plot(lam, error, 'b-',label='Error')
        ax2.scatter(lam[peaks],error[peaks],label='peaks')
        ax2.plot(lam, np.degrees(angle_error), 'y-',label='Normal Error')
        if draw_speed_max is None:
            draw_speed_max=max(speed)*1.05
        ax1.axis(ymin=0,ymax=draw_speed_max)
        if draw_error_max is None:
            draw_error_max=max(error)*1.05
        ax2.axis(ymin=0,ymax=draw_error_max)

        ax1.set_xlabel('lambda (mm)')
        ax1.set_ylabel('Speed/lamdot (mm/s)', color='g')
        ax2.set_ylabel('Error/Normal Error (mm/deg)', color='b')
        plt.title("Speed and Error Plot")
        ax1.legend(loc=0)
        ax2.legend(loc=0)

        # save fig
        plt.legend()
        plt.savefig(ilc_output+'iteration_'+str(i))
        plt.clf()
        # plt.show()
        # save bp
        df=DataFrame({'primitives':primitives,'points':p_bp,'q_bp':q_bp})
        df.to_csv(ilc_output+'command_'+str(i)+'.csv',header=True,index=False)

        if max(error)<error_tol and max(np.rad2deg(angle_error))<angerror_tol and (std_speed/ave_speed*100)<velstd_tol:
            print("Tolerance Satisfied")
            break
        
        ##########################################calculate gradient######################################
        ######gradient calculation related to nearest 3 points from primitive blended trajectory, not actual one
        ###restore trajectory from primitives
        curve_interp, curve_R_interp, curve_js_interp, breakpoints_blended=form_traj_from_bp(q_bp,primitives,robot)

        curve_js_blended,curve_blended,curve_R_blended=blend_js_from_primitive(curve_interp, curve_js_interp, breakpoints_blended, primitives,robot,zone=10)
        # fanuc blend in cart space
        # curve_js_blended,curve_blended,curve_R_blended=blend_cart_from_primitive(curve_interp,curve_R_interp,curve_js_interp,breakpoints_blended,primitives,robot,ave_speed)

        # plt.figure()
        # ax = plt.axes(projection='3d')
        # ax.plot3D(curve_interp[:,0], curve_interp[:,1],curve_interp[:,2], 'blue',label='Motion Cmd')
        # ax.plot3D(curve_blended[:,0], curve_blended[:,1],curve_blended[:,2], 'red',label='Blend')
        # #plot execution curve
        # ax.plot3D(curve_exe[:,0], curve_exe[:,1],curve_exe[:,2], 'green',label='Executed Motion')
        # ax.view_init(elev=40, azim=-145)
        # ax.set_title('Cartesian Interpolation using Motion Cmd')
        # ax.set_xlabel('x-axis (mm)')
        # ax.set_ylabel('y-axis (mm)')
        # ax.set_zlabel('z-axis (mm)')
        # plt.show()
        # exit()
        # plt.plot(curve_interp[:,0],curve_interp[:,1])
        # plt.plot(curve_blended[:,0],curve_blended[:,1])
        # plt.axis('equal')
        # plt.show()

        for peak in peaks:
            ######gradient calculation related to nearest 3 points from primitive blended trajectory, not actual one
            _,peak_error_curve_idx=calc_error(curve_exe[peak],curve[:,:3])  # index of original curve closest to max error point

            ###get closest to worst case point on blended trajectory
            _,peak_error_curve_blended_idx=calc_error(curve_exe[peak],curve_blended)

            ###############get numerical gradient#####
            ###find closest 3 breakpoints
            order=np.argsort(np.abs(breakpoints_blended-peak_error_curve_blended_idx))
            breakpoint_interp_2tweak_indices=order[:3]

            de_dp=ilc.get_gradient_from_model_xyz_fanuc(p_bp,q_bp,breakpoints_blended,curve_blended,peak_error_curve_blended_idx,robot.fwd(curve_exe_js[peak]),curve[peak_error_curve_idx,:3],breakpoint_interp_2tweak_indices,ave_speed)
            p_bp, q_bp=ilc.update_bp_xyz(p_bp,q_bp,de_dp,error[peak],breakpoint_interp_2tweak_indices,alpha=alpha)

def main():
    
    ### set curve type and speed
    parser = argparse.ArgumentParser(description='FANUC Max Gradient')
    parser.add_argument("--curve", type=str, help="curve_1 or curve_2. Default: curve_1")
    parser.add_argument("--speed", type=int, help="Command Speed (1~2000). Default: curve_1 , curve_2 ")
    parser.add_argument("--iteration", type=int, help="Iterations. Default: 10.")
    args,_ = parser.parse_known_args()

    all_curves=['curve_1','curve_2']
    if args.curve not in all_curves:
        print("Choices of curves:",all_curves,". Using default curve, curve_1")
        args.curve = 'curve_1'
    if args.speed is None:
        if args.curve == 'curve_1':
            args.speed=300
        elif args.curve == 'curve_2':
            args.speed=600
    s = min([max([args.speed,1]),2000]) # speed: 1~2000
    print("Multi-Peak Max Gradient. Curve:",args.curve,", Command Speed:",s)
    ##############################

    ### dataset folders
    dataset=args.curve+'/'
    data_dir="../data/"+dataset+'fanuc/'
    ilc_output='max_gradient/fanuc/'+args.curve+'_speed_'+str(s)+'/'
    Path(ilc_output).mkdir(exist_ok=True)
    
    ### curve in joint space and base frame
    curve_js=read_csv(data_dir+'Curve_js.csv',header=None).values
    curve = read_csv(data_dir+"Curve_in_base_frame.csv",header=None).values
    curve_normal=curve[:,3:]
    
    if args.iteration is None: # iteration of gradient descent
        iteration=10
    else:
        iteration=args.iteration
    print("Total Iterations:",iteration)

    robot=m710ic(d=50) # FANUC m710ic-70

    max_grad_descent(data_dir,robot,s,curve,curve_js)
    

if __name__ == "__main__":
    main()
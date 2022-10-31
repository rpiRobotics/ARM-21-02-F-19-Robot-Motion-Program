import numpy as np
from copy import deepcopy
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
    error_tol=0.5,angerror_tol=3,velstd_tol=5,iteration_max=100,save_all_file=False,save_ls=False,save_name=''):

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

    alpha = 0.5 # for gradient descent
    alpha_error_dir = 0.8 # for pushing in error direction

    ### speed,zone
    s = velocity
    z = 100 # zone (corner path), CNT100

    ### Gradient descent parameters
    multi_peak_threshold=0.2 # decreasing peak higher than threshold
    alpha = 0.5 # gradient descentz step size
    
    q_bp_start = q_bp[0][0]
    q_bp_end = q_bp[-1][-1]
    primitives,p_bp,q_bp=extend_start_end(robot,q_bp,primitives,breakpoints,p_bp,extension_d=100)

    ## calculate step at start and end
    step_start1=None
    step_end1=None
    for i in range(len(q_bp)):
        if np.all(q_bp[i][0]==q_bp_start):
            step_start1=i
        if np.all(q_bp[i][-1]==q_bp_end):
            step_end1=i

    assert step_start1 is not None,'Cant find step start'
    assert step_end1 is not None,'Cant find step start'
    print(step_start1,step_end1)

    ###ilc toolbox def
    ilc=ilc_toolbox(robot,primitives)

    ###TODO: extension fix start point, moveC support
    ms = MotionSendFANUC()
    draw_error_max=None
    draw_speed_max=None
    max_gradient_descent_flag = False
    max_error_prev = 999999999
    for i in range(iteration_max):
        
        ###execute,curve_fit_js only used for orientation
        logged_data=ms.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,s,z,save_ls,ilc_output+save_name)

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
        print('Iteration:',i,', Max Error:',max(error),', Ave. Speed:',ave_speed, ', Speed Std:',std_speed, 'Speed Std/Ave:',(std_speed*100/ave_speed),'%')
        #############################error peak detection###############################
        peaks,_=find_peaks(error,height=multi_peak_threshold,prominence=0.05,distance=20/(lam[int(len(lam)/2)]-lam[int(len(lam)/2)-1]))		###only push down peaks higher than height, distance between each peak is 20mm, threshold to filter noisy peaks

        if len(peaks)==0 or np.argmax(error) not in peaks:
            peaks=np.append(peaks,np.argmax(error))
        
        ##############################plot error#####################################
        try:
            plt.close(fig)
        except:
            pass
        # fig, ax1 = plt.subplots()
        fig, ax1 = plt.subplots(figsize=(6,4))
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
        # ax1.legend(loc=0)
        # ax2.legend(loc=0)
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax1.legend(h1+h2, l1+l2, loc=1)

        #### save all things
        if save_all_file:
            # save fig
            plt.savefig(ilc_output+'iteration_'+str(i))
            plt.savefig(ilc_output+'final_iteration')
            plt.clf()
        else:
            fig.canvas.manager.window.move(2818,120)
            plt.show(block=False)
            plt.pause(0.1)

        # save bp
        if save_all_file:
            df=DataFrame({'primitives':primitives,'points':p_bp,'q_bp':q_bp})
            df.to_csv(ilc_output+'command_'+str(i)+'.csv',header=True,index=False)
            DataFrame(curve_exe_js).to_csv(ilc_output+'curve_js_exe.csv',header=False,index=False)
            np.save(ilc_output+'final_speed.npy',speed)
            np.save(ilc_output+'final_error.npy',error)
            np.save(ilc_output+'final_ang_error.npy',angle_error)


        if max(error)<error_tol and max(np.rad2deg(angle_error))<angerror_tol and (std_speed/ave_speed*100)<velstd_tol:
            print("Tolerance Satisfied")
            # time.sleep(5)
            break
        if max(error)<error_tol and max(np.rad2deg(angle_error))<angerror_tol:
            print("Speed Tolerance Not Satisfied")
            # time.sleep(5)
            break
        
        # if max error does not decrease, use multi-peak max gradient descent
        if max(error) > max_error_prev:
            max_gradient_descent_flag = True

        p_bp1_update = deepcopy(p_bp)
        q_bp1_update = deepcopy(q_bp)
        if not max_gradient_descent_flag: # update through push in error direction
            ##########################################calculate error direction and push######################################
            ### interpolate curve (get gradient direction)
            curve_target = np.zeros((len(curve_exe), 3))
            curve_target_R = np.zeros((len(curve_exe), 3))
            for j in range(len(curve_exe)):
                dist = np.linalg.norm(curve[:,:3] - curve_exe[j], axis=1)
                closest_point_idx = np.argmin(dist)
                curve_target[j, :] = curve[closest_point_idx, :3]
                curve_target_R[j, :] = curve[closest_point_idx, 3:]

            ### get error (and transfer into robot1 frame)
            error1 = []
            angle_error1 = []
            for j in range(len(curve_exe)):
                ## calculate error
                error1.append(curve_target[j]-curve_exe[j])
                # angle_error1.append(get_angle(curve_exe_R1[j][:,-1], this_curve_target_R1))
                angle_error1.append(curve_target_R[j]-curve_exe_R[j][:,-1])

            ### get closets bp index
            p_bp1_error_dir=[]
            p_bp1_ang_error_dir=[]
            # find error direction
            for j in range(step_start1, step_end1+1): # exclude first and the last bp and the bp for extension
                this_p_bp = p_bp[j]
                closest_point_idx = np.argmin(np.linalg.norm(curve_target - this_p_bp, axis=1))
                error_dir = error1[closest_point_idx]
                p_bp1_error_dir.append(error_dir)
                ang_error_dir = angle_error1[closest_point_idx]
                p_bp1_ang_error_dir.append(ang_error_dir)
            
            ### update all p_bp1
            for j in range(step_start1,step_end1+1):
                p_bp1_update[j][-1] = np.array(p_bp[j][-1]) + alpha_error_dir*p_bp1_error_dir[j-step_start1]
                bp1_R = robot.fwd(q_bp[j][-1]).R
                # bp1_R[:,-1] = (bp1_R[:,-1] + alpha_error_dir*p_bp1_ang_error_dir[j-step_start1])/np.linalg.norm((bp1_R[:,-1] + alpha_error_dir*p_bp1_ang_error_dir[j-step_start1]))
                q_bp1_update[j][-1] = car2js(robot, q_bp[j][-1], p_bp1_update[j][-1], bp1_R)[0]
            
            p_bp = deepcopy(p_bp1_update)
            q_bp = deepcopy(q_bp1_update)
        else:
            ##########################################calculate gradient######################################
            ######gradient calculation related to nearest 3 points from primitive blended trajectory, not actual one
            ###restore trajectory from primitives
            curve_interp, curve_R_interp, curve_js_interp, breakpoints_blended=form_traj_from_bp(q_bp,primitives,robot)

            curve_js_blended,curve_blended,curve_R_blended=blend_js_from_primitive(curve_interp, curve_js_interp, breakpoints_blended, primitives,robot,zone=10)

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
    
    return curve_exe_js,speed,error,np.rad2deg(angle_error),breakpoints,primitives,q_bp,p_bp

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
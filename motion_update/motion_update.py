from .ilc_toolbox import *
from realrobot import *
from dual_arm import *
from pathlib import Path
import traceback
from copy import deepcopy

def motion_program_update(filepath,robot,robot_ip,robotMotionSend,vel,desired_curve_filename,desired_curvejs_filename,\
    err_tol,angerr_tol,velstd_tol,extstart,extend,realrobot):
    try:
        curve = read_csv(desired_curve_filename,header=None).values
        curve=np.array(curve)
        curve_js = read_csv(desired_curvejs_filename,header=None).values
        curve_js=np.array(curve_js)

        # return error_descent_fanuc(filepath,robot,robot_ip,robotMotionSend,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls',extstart=extstart,extend=extend)
        if 'ABB' in robot.def_path:
            return error_descent_abb(filepath,robot,robot_ip,robotMotionSend,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls',realrobot=realrobot)
        if 'FANUC' in robot.def_path:
            return error_descent_fanuc(filepath,robot,robot_ip,robotMotionSend,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls',extstart=extstart,extend=extend,realrobot=realrobot)
    except:
        traceback.print_exc()

def motion_program_update_dual(filepath,robot1,robot2,robot_ip,robotMotionSend,vel,desired_curve_filename,desired_curvejs1_filename,desired_curvejs2_filename,\
    err_tol,angerr_tol,velstd_tol,extstart,extend,realrobot,utool2=None):

    try:
        curve = read_csv(desired_curve_filename,header=None).values
        curve=np.array(curve)
        curve_js1 = np.array(read_csv(desired_curvejs1_filename,header=None).values)
        curve_js2 = np.array(read_csv(desired_curvejs2_filename,header=None).values)

        # return error_descent_fanuc(filepath,robot,robot_ip,robotMotionSend,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls',extstart=extstart,extend=extend)
        if 'ABB' in robot1.def_path:
            return error_descent_abb_dual(filepath,robot1,robot2,robot_ip,robotMotionSend,vel,curve,curve_js1,curve_js2,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls',extstart=extstart,extend=extend,realrobot=realrobot,utool2=utool2)
        if 'FANUC' in robot1.def_path:
            return error_descent_fanuc_dual(filepath,robot1,robot2,robot_ip,robotMotionSend,vel,curve,curve_js1,curve_js2,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls',extstart=extstart,extend=extend,realrobot=realrobot,utool2=utool2)
    except:
        traceback.print_exc()

def error_descent_abb(filepath,robot,robot_ip,robotMotionSend,velocity,desired_curve,desired_curve_js,\
    error_tol=0.5,angerror_tol=3,velstd_tol=5,iteration_max=100,save_all_file=False,save_ls=False,save_name='',realrobot=False):

    curve=desired_curve
    curve_js=desired_curve_js

    ilc_output=filepath+'/result_speed_'+str(velocity)+'/'
    Path(ilc_output).mkdir(exist_ok=True)

    ms = robotMotionSend('http://'+robot_ip+':80')

    breakpoints,primitives,p_bp,q_bp=ms.extract_data_from_cmd(filepath+'/command.csv')

    alpha = 0.5 # for gradient descent
    alpha_error_dir = 0.8 # for pushing in error direction

    ### speed,zone
    s = speeddata(velocity,9999999,9999999,999999)
    zone=100
    z = zonedata(False,zone,1.5*zone,1.5*zone,0.15*zone,1.5*zone,0.15*zone) # zone (corner path), CNT100

    ### Gradient descent parameters
    multi_peak_threshold=0.2 # decreasing peak higher than threshold
    alpha = 0.5 # gradient descentz step size
    
    p_bp,q_bp=ms.extend(robot,q_bp,primitives,breakpoints,p_bp)  

    ###ilc toolbox def
    ilc=ilc_toolbox(robot,primitives)

    draw_error_max=None
    draw_speed_max=None
    max_gradient_descent_flag = False
    max_error_prev = 999999999
    for i in range(iteration_max):

        if realrobot:
            curve_js_all_new, curve_exe_js, timestamp=average_N_exe(ms,robot,primitives,breakpoints,p_bp,q_bp,s,z,curve,log_path="",N=5)
            lam, curve_exe, curve_exe_R, speed=logged_data_analysis(robot,timestamp,curve_exe_js)
        else:
            ###execute,curve_fit_js only used for orientation       ###TODO: add save_ls
            log_results=ms.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,s,z)#,save_ls,ilc_output+save_name)
            ##############################data analysis#####################################
            lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.logged_data_analysis(robot,log_results)

        #############################chop extension off##################################
        lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.chop_extension(curve_exe, curve_exe_R,curve_exe_js, speed, timestamp,curve[0,:3],curve[-1,:3])
        

        ave_speed=np.mean(speed)
        std_speed=np.std(speed)

        ##############################calcualte error########################################
        error,angle_error=calc_all_error_w_normal(curve_exe,curve[:,:3],curve_exe_R[:,:,-1],curve[:,3:])
        print('Iteration:',i,', Max Error:',max(error),', Ave. Speed:',ave_speed, ', Speed Std:',std_speed, 'Speed Std/Ave:',(std_speed*100/ave_speed),'%')
        #############################error peak detection###############################
        peaks,_=find_peaks(error,height=multi_peak_threshold,prominence=0.05,distance=20/(lam[int(len(lam)/2)]-lam[int(len(lam)/2)-1]))     ###only push down peaks higher than height, distance between each peak is 20mm, threshold to filter noisy peaks

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

        if not max_gradient_descent_flag: # update through push in error direction
            ##########################################calculate error direction and push######################################
            error_bps_v,error_bps_w=ilc.get_error_direction(curve,p_bp,q_bp,curve_exe,curve_exe_R)
            p_bp, q_bp=ilc.update_error_direction(curve,p_bp,q_bp,error_bps_v,error_bps_w)
        else:
            ##########################################calculate gradient######################################
            ######gradient calculation related to nearest 3 points from primitive blended trajectory, not actual one
            ###restore trajectory from primitives
            print('MAX GRADIENT')
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
                breakpoint_interp_2tweak_indices=order[:2]

                peak_pose=robot.fwd(curve_exe_js[peak])
                ##################################################################XYZ Gradient######################################################################
                de_dp=ilc.get_gradient_from_model_xyz(p_bp,q_bp,breakpoints_blended,curve_blended,peak_error_curve_blended_idx,peak_pose,curve[peak_error_curve_idx,:3],breakpoint_interp_2tweak_indices)
                p_bp, q_bp=ilc.update_bp_xyz(p_bp,q_bp,de_dp,error[peak],breakpoint_interp_2tweak_indices)

                ##################################################################Ori Gradient######################################################################
                de_ori_dp=ilc.get_gradient_from_model_ori(p_bp,q_bp,breakpoints_blended,curve_R_blended,peak_error_curve_blended_idx,peak_pose,curve[peak_error_curve_idx,3:],breakpoint_interp_2tweak_indices)
                q_bp=ilc.update_bp_ori(p_bp,q_bp,de_ori_dp,angle_error[peak],breakpoint_interp_2tweak_indices)
    
    return curve_exe_js,speed,error,np.rad2deg(angle_error),breakpoints,primitives,q_bp,p_bp


def error_descent_fanuc(filepath,robot,robot_ip,robotMotionSend,velocity,desired_curve,desired_curve_js,\
    error_tol=0.5,angerror_tol=3,velstd_tol=5,iteration_max=100,save_all_file=False,save_ls=False,save_name='',extstart=100,extend=100,realrobot=False):

    curve=desired_curve
    curve_js=desired_curve_js

    ilc_output=filepath+'/result_speed_'+str(velocity)+'/'
    Path(ilc_output).mkdir(exist_ok=True)

    ms = robotMotionSend(group=1,uframe=1,utool=2,robot_ip=robot_ip,robot1=robot)

    breakpoints,primitives,p_bp,q_bp=ms.extract_data_from_cmd(filepath+'/command.csv')

    alpha = 0.5 # for gradient descent
    alpha_error_dir = 0.8 # for pushing in error direction

    ### speed,zone
    s = velocity
    z = 100 # zone (corner path), CNT100

    ### Gradient descent parameters
    multi_peak_threshold=0.2 # decreasing peak higher than threshold
    alpha = 0.5 # gradient descentz step size
    
    primitives,p_bp,q_bp=ms.extend_start_end(robot,q_bp,primitives,breakpoints,p_bp,extension_start=extstart,extension_end=extend)

    ###ilc toolbox def
    ilc=ilc_toolbox(robot,primitives)

    draw_error_max=None
    draw_speed_max=None
    max_gradient_descent_flag = False
    max_error_prev = 999999999
    for i in range(iteration_max):
        
        if realrobot:
            lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=average_N_exe_fanuc(ms,robot,primitives,breakpoints,p_bp,q_bp,s,z,curve,log_path="",N=5)
        else:
            ###execute,curve_fit_js only used for orientation
            logged_data=ms.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,s,z)
            df = read_csv(StringIO(logged_data.decode('utf-8')))
            ##############################data analysis#####################################
            lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.logged_data_analysis(robot,df)

        #############################chop extension off##################################
        lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp=ms.chop_extension(curve_exe, curve_exe_R,curve_exe_js, speed, timestamp,curve[:,:3],curve[:,3:])
        ave_speed=np.mean(speed)
        std_speed=np.std(speed)

        ##############################calcualte error########################################
        error,angle_error=calc_all_error_w_normal(curve_exe,curve[:,:3],curve_exe_R[:,:,-1],curve[:,3:])
        print('Iteration:',i,', Max Error:',max(error), ', Max Ang Error:',max(np.rad2deg(angle_error)),', Ave. Speed:',ave_speed, ', Speed Std:',std_speed, 'Speed Std/Ave:',(std_speed*100/ave_speed),'%')
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
        if max(error) > max_error_prev and max_gradient_descent_flag:
            return curve_exe_js,speed,error,np.rad2deg(angle_error),breakpoints,primitives,q_bp,p_bp

        if max(error) > max_error_prev:
            max_gradient_descent_flag = True

        if not max_gradient_descent_flag: # update through push in error direction
            ##########################################calculate error direction and push######################################
            error_bps_v,error_bps_w=ilc.get_error_direction(curve,p_bp,q_bp,curve_exe,curve_exe_R)
            p_bp, q_bp=ilc.update_error_direction(curve,p_bp,q_bp,error_bps_v,error_bps_w,extension=False)
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
                breakpoint_interp_2tweak_indices=order[:2]

                peak_pose=robot.fwd(curve_exe_js[peak])
                ##################################################################XYZ Gradient######################################################################
                de_dp=ilc.get_gradient_from_model_xyz(p_bp,q_bp,breakpoints_blended,curve_blended,peak_error_curve_blended_idx,peak_pose,curve[peak_error_curve_idx,:3],breakpoint_interp_2tweak_indices)
                p_bp, q_bp=ilc.update_bp_xyz(p_bp,q_bp,de_dp,error[peak],breakpoint_interp_2tweak_indices)

                ##################################################################Ori Gradient######################################################################
                de_ori_dp=ilc.get_gradient_from_model_ori(p_bp,q_bp,breakpoints_blended,curve_R_blended,peak_error_curve_blended_idx,peak_pose,curve[peak_error_curve_idx,3:],breakpoint_interp_2tweak_indices)
                p_bp, q_bp=ilc.update_bp_xyz(p_bp,q_bp,de_dp,error[peak],breakpoint_interp_2tweak_indices,alpha=alpha)
    
    return curve_exe_js,speed,error,np.rad2deg(angle_error),breakpoints,primitives,q_bp,p_bp

def error_descent_fanuc_dual(filepath,robot1,robot2,robot_ip,robotMotionSend,velocity,desired_curve,desired_curve_js1,desired_curve_js2,\
    error_tol=0.5,angerror_tol=3,velstd_tol=5,iteration_max=100,save_all_file=False,save_ls=False,save_name='',extstart=100,extend=100,realrobot=False,utool2=2):

    ## desired curve
    relative_path=desired_curve
    ## output directory
    if realrobot:
        ilc_output=filepath+'/result_speed_'+str(velocity)+'_realrobot/'
    else:
        ilc_output=filepath+'/result_speed_'+str(velocity)+'/'
    Path(ilc_output).mkdir(exist_ok=True)
    ## robot2 base
    base2_T=robot2.base_H
    base2_R=base2_T[:3,:3]
    base2_p=base2_T[:3,-1]

    # fanuc motion send tool
    ms = robotMotionSend(group=1,uframe=1,utool=2,robot_ip=robot_ip,robot1=robot1,robot2=robot2,utool2=utool2)
    
    s=velocity # mm/sec in leader frame
    z=100 # CNT100

    breakpoints1,primitives1,p_bp1,q_bp1,_=ms.extract_data_from_cmd(filepath+'/command1.csv')
    breakpoints2,primitives2,p_bp2,q_bp2,_=ms.extract_data_from_cmd(filepath+'/command2.csv')
    p_bp1,q_bp1,p_bp2,q_bp2=ms.extend_dual(ms.robot1,p_bp1,q_bp1,primitives1,ms.robot2,p_bp2,q_bp2,primitives2,breakpoints1,0,extension_start=extstart,extension_end=extend)  ## curve_1, greedy 0.2

    ilc=ilc_toolbox([robot1,robot2],[primitives1,primitives2])

    max_error_tolerance=error_tol
    max_ang_error_tolerance=np.radians(angerror_tol)

    multi_peak_threshold=0.2
    ###TODO: extension fix start point, moveC support
    draw_speed_max=None
    draw_error_max=None
    max_error = 999
    error_localmin_flag=False
    use_grad=False
    for i in range(iteration_max):

        if realrobot:
            lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe, relative_path_exe_R = \
                average_N_exe_multimove_fanuc(ms,robot1,robot2,base2_R,base2_p,primitives1,primitives2,breakpoints1,p_bp1,p_bp2,q_bp1,q_bp2,s,z,log_path='',N=2)
        else:
            ###execution with plant
            logged_data=ms.exec_motions_multimove(robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,s,z)
            StringData=StringIO(logged_data.decode('utf-8'))
            df = read_csv(StringData, sep =",")
            ##############################data analysis#####################################
            lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe, relative_path_exe_R = ms.logged_data_analysis_multimove(df,base2_R,base2_p,realrobot=False)
        
        #############################chop extension off##################################
        lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe, relative_path_exe_R=\
            ms.chop_extension_dual(lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R,relative_path[0,:3],relative_path[-1,:3])
        ave_speed=np.mean(speed)
        std_speed=np.std(speed)
        ##############################calcualte error########################################
        error,angle_error=calc_all_error_w_normal(relative_path_exe,relative_path[:,:3],relative_path_exe_R[:,:,-1],relative_path[:,3:])
        print('Iteration:',i,', Max Error:',max(error),'Ave. Speed:',ave_speed,'Std. Speed:',np.std(speed),'Std/Ave (%):',np.std(speed)/ave_speed*100)
        print('Max Speed:',max(speed),'Min Speed:',np.min(speed),'Ave. Error:',np.mean(error),'Min Error:',np.min(error),"Std. Error:",np.std(error))
        print('Max Ang Error:',max(np.degrees(angle_error)),'Min Ang Error:',np.min(np.degrees(angle_error)),'Ave. Ang Error:',np.mean(np.degrees(angle_error)),"Std. Ang Error:",np.std(np.degrees(angle_error)))
        print("===========================================")
        #############################error peak detection###############################
        find_peak_dist = 20/(lam[int(len(lam)/2)]-lam[int(len(lam)/2)-1])
        if find_peak_dist<1:
            find_peak_dist=1
        peaks,_=find_peaks(error,height=multi_peak_threshold,prominence=0.05,distance=find_peak_dist)		###only push down peaks higher than height, distance between each peak is 20mm, threshold to filter noisy peaks
        if len(peaks)==0 or np.argmax(error) not in peaks:
            peaks=np.append(peaks,np.argmax(error))

        # peaks=np.array([np.argmax(error)])
        ##############################plot error#####################################
        fig, ax1 = plt.subplots()
        ax2 = ax1.twinx()
        ax1.plot(lam, speed, 'g-', label='Speed')
        ax2.plot(lam, error, 'b-',label='Error')
        ax2.scatter(lam[peaks],error[peaks],label='peaks')
        ax2.plot(lam, np.degrees(angle_error), 'y-',label='Normal Error')
        if draw_speed_max is None:
            draw_speed_max=max(speed)*1.05
        if max(speed) >= draw_speed_max or max(speed) < draw_speed_max*0.1:
            draw_speed_max=max(speed)*1.05
        ax1.axis(ymin=0,ymax=draw_speed_max)
        if draw_error_max is None:
            draw_error_max=max(error)*1.05
        if max(error) >= draw_error_max or max(error) < draw_error_max*0.1:
            draw_error_max=max(error)*1.05
        ax2.axis(ymin=0,ymax=draw_error_max)
        ax1.set_xlabel('lambda (mm)')
        ax1.set_ylabel('Speed/lamdot (mm/s)', color='g')
        ax2.set_ylabel('Error/Normal Error (mm/deg)', color='b')
        plt.title("Speed and Error Plot")
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
            df=DataFrame({'primitives':primitives1,'points':p_bp1,'q_bp':q_bp1})
            df.to_csv(ilc_output+'command_arm1_'+str(i)+'.csv',header=True,index=False)
            df=DataFrame({'primitives':primitives2,'points':p_bp2,'q_bp':q_bp2})
            df.to_csv(ilc_output+'command_arm2_'+str(i)+'.csv',header=True,index=False)
            DataFrame(curve_exe_js1).to_csv(ilc_output+'curve_js1_exe.csv',header=False,index=False)
            DataFrame(curve_exe_js2).to_csv(ilc_output+'curve_js2_exe.csv',header=False,index=False)
            np.save(ilc_output+'final_speed.npy',speed)
            np.save(ilc_output+'final_error.npy',error)
            np.save(ilc_output+'final_ang_error.npy',angle_error)

        ###########################plot for verification###################################
        # p_bp_relative,_=ms.form_relative_path(np.squeeze(q_bp1),np.squeeze(q_bp2),base2_R,base2_p)

        if max(error)>max_error and error_localmin_flag:
            print("Can't decrease anymore")
            break

        if max(error)>max_error:
            print("Use grad")
            error_localmin_flag=True
            use_grad=True
        
        if max(error)<max_error:
            error_localmin_flag=False

        max_error=max(error)

        if max(error) < max_error_tolerance and max(angle_error)<max_ang_error_tolerance and (std_speed/ave_speed*100)<velstd_tol:
            print("Tolerance Satisfied")
            break
    
        if max(error) < max_error_tolerance and max(angle_error)<max_ang_error_tolerance:
            print("Speed Tolerance Not Satisfied")
            break

        if use_grad:
            ##########################################calculate gradient for peaks######################################
            ###restore trajectory from primitives
            curve_interp1, curve_R_interp1, curve_js_interp1, breakpoints_blended=form_traj_from_bp(q_bp1,primitives1,robot1)
            curve_interp2, curve_R_interp2, curve_js_interp2, breakpoints_blended=form_traj_from_bp(q_bp2,primitives2,robot2)
            curve_js_blended1,curve_blended1,curve_R_blended1=blend_js_from_primitive(curve_interp1, curve_js_interp1, breakpoints_blended, primitives1,robot1,zone=10)
            curve_js_blended2,curve_blended2,curve_R_blended2=blend_js_from_primitive(curve_interp2, curve_js_interp2, breakpoints_blended, primitives2,robot2,zone=10)

            ###establish relative trajectory from blended trajectory
            relative_path_blended,relative_path_blended_R=ms.form_relative_path(curve_js_blended1,curve_js_blended2,base2_R,base2_p)

            all_new_bp=[]
            for peak in peaks:
                ######gradient calculation related to nearest 3 points from primitive blended trajectory, not actual one
                _,peak_error_curve_idx=calc_error(relative_path_exe[peak],relative_path[:,:3])  # index of original curve closest to max error point

                ###get closest to worst case point on blended trajectory
                _,peak_error_curve_blended_idx=calc_error(relative_path_exe[peak],relative_path_blended)

                ###############get numerical gradient#####
                ###find closest 3 breakpoints
                order=np.argsort(np.abs(breakpoints_blended-peak_error_curve_blended_idx))
                breakpoint_interp_2tweak_indices=order[:3]

                de_dp=ilc.get_gradient_from_model_xyz_dual(\
                    [p_bp1,p_bp2],[q_bp1,q_bp2],breakpoints_blended,[curve_blended1,curve_blended2],peak_error_curve_blended_idx,[curve_exe_js1[peak],curve_exe_js2[peak]],relative_path[peak_error_curve_idx,:3],breakpoint_interp_2tweak_indices)


                p_bp1_new, q_bp1_new,p_bp2_new,q_bp2_new=ilc.update_bp_xyz_dual([p_bp1,p_bp2],[q_bp1,q_bp2],de_dp,error[peak],breakpoint_interp_2tweak_indices,alpha=0.5)

                ###update
                p_bp1=p_bp1_new
                q_bp1=q_bp1_new
                p_bp2=p_bp2_new
                q_bp2=q_bp2_new
        else:
            error_bps_v1,error_bps_w1,error_bps_v2,error_bps_w2=ilc.get_error_direction_dual(relative_path,p_bp1,q_bp1,p_bp2,q_bp2,relative_path_exe,relative_path_exe_R,curve_exe1,curve_exe_R1,curve_exe2,curve_exe_R2)
            error_bps_w1=np.zeros(error_bps_w1.shape)
            error_bps_w2=np.zeros(error_bps_w2.shape)
            # error_bps_v1=np.zeros(error_bps_v1.shape)
            # error_bps_v2=np.zeros(error_bps_v2.shape)

            gamma_v=0.2
            gamma_w=0.1
            p_bp1_origin=deepcopy(p_bp1)
            p_bp2_origin=deepcopy(p_bp2)
            q_bp1_origin=deepcopy(q_bp1)
            q_bp2_origin=deepcopy(q_bp2)
            while True:
                try:
                    p_bp1, q_bp1, p_bp2, q_bp2=ilc.update_error_direction_dual(relative_path,p_bp1_origin,q_bp1_origin,p_bp2_origin,q_bp2_origin,error_bps_v1,error_bps_w1,error_bps_v2,error_bps_w2,gamma_v,gamma_w)
                    break
                except IndexError:
                    gamma_v*=0.75
                    gamma_w*=0.75
                    if gamma_w<0.02:
                        # error_localmin_flag=True
                        use_grad=True
                        break
    
    return curve_exe_js1,curve_exe_js2,speed,error,np.rad2deg(angle_error),breakpoints1,primitives1,q_bp1,p_bp1,primitives2,q_bp2,p_bp2




def error_descent_abb_dual(filepath,robot1,robot2,robot_ip,robotMotionSend,velocity,desired_curve,curve_js1,curve_js2,\
    error_tol=0.5,angerror_tol=3,velstd_tol=5,iteration_max=100,save_all_file=False,save_ls=False,save_name='',extstart=100,extend=100,realrobot=False,utool2=2):

    ## desired curve
    relative_path=desired_curve
    lam_relative_path=calc_lam_cs(relative_path)
    lam1=calc_lam_js(curve_js1,robot1)
    lam2=calc_lam_js(curve_js2,robot2)
    ## output directory
    if realrobot:
        ilc_output=filepath+'/result_speed_'+str(velocity)+'_realrobot/'
    else:
        ilc_output=filepath+'/result_speed_'+str(velocity)+'/'
    Path(ilc_output).mkdir(exist_ok=True)
    ## robot2 base
    base2_T=robot2.base_H
    base2_R=base2_T[:3,:3]
    base2_p=base2_T[:3,-1]


    # abb motion send tool
    ms = robotMotionSend('http://'+robot_ip+':80')
    breakpoints1,primitives1,p_bp1,q_bp1=ms.extract_data_from_cmd(filepath+'/command1.csv')
    breakpoints2,primitives2,p_bp2,q_bp2=ms.extract_data_from_cmd(filepath+'/command2.csv')


    s1_all,s2_all=calc_individual_speed(velocity,lam1,lam2,lam_relative_path,breakpoints1)
    v2_all=[]
    for i in range(len(breakpoints1)):
        v2_all.append(speeddata(s2_all[i],9999999,9999999,999999))
        # v2_all.append(v5000)

    
    p_bp1,q_bp1,p_bp2,q_bp2=ms.extend_dual(robot1,p_bp1,q_bp1,primitives1,robot2,p_bp2,q_bp2,primitives2,breakpoints1)

    ilc=ilc_toolbox([robot1,robot2],[primitives1,primitives2])

    max_error_tolerance=error_tol
    max_ang_error_tolerance=np.radians(angerror_tol)

    multi_peak_threshold=0.2
    ###TODO: extension fix start point, moveC support
    draw_speed_max=None
    draw_error_max=None
    max_error = 999
    error_localmin_flag=False
    use_grad=False
    for i in range(iteration_max):

        if realrobot:
            curve_js_all_new, avg_curve_js, timestamp_d=average_N_exe_multimove(ms,breakpoints1,robot1,primitives1,p_bp1,q_bp1,vmax,z50,robot2,primitives2,p_bp2,q_bp2,v2_all,z50,relative_path,safeq1=None,safeq2=None,log_path='',N=5)
            ###calculat data with average curve
            lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe, relative_path_exe_R =\
                logged_data_analysis_multimove(robot1,robot2,timestamp_d,avg_curve_js)
        else:
            ###execution with plant
            log_results=ms.exec_motions_multimove(robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,vmax,v2_all,z50,z50)
            lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R = ms.logged_data_analysis_multimove(log_results,robot1,robot2,realrobot=True)

        #############################chop extension off##################################
        lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe, relative_path_exe_R=\
            ms.chop_extension_dual(lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R,relative_path[0,:3],relative_path[-1,:3])
        
        ave_speed=np.mean(speed)
        std_speed=np.std(speed)
        ##############################calcualte error########################################
        error,angle_error=calc_all_error_w_normal(relative_path_exe,relative_path[:,:3],relative_path_exe_R[:,:,-1],relative_path[:,3:])
        print('Iteration:',i,', Max Error:',max(error),'Ave. Speed:',ave_speed,'Std. Speed:',np.std(speed),'Std/Ave (%):',np.std(speed)/ave_speed*100)
        print('Max Speed:',max(speed),'Min Speed:',np.min(speed),'Ave. Error:',np.mean(error),'Min Error:',np.min(error),"Std. Error:",np.std(error))
        print('Max Ang Error:',max(np.degrees(angle_error)),'Min Ang Error:',np.min(np.degrees(angle_error)),'Ave. Ang Error:',np.mean(np.degrees(angle_error)),"Std. Ang Error:",np.std(np.degrees(angle_error)))
        print("===========================================")
        #############################error peak detection###############################
        find_peak_dist = 20/(lam[int(len(lam)/2)]-lam[int(len(lam)/2)-1])
        if find_peak_dist<1:
            find_peak_dist=1
        peaks,_=find_peaks(error,height=multi_peak_threshold,prominence=0.05,distance=find_peak_dist)       ###only push down peaks higher than height, distance between each peak is 20mm, threshold to filter noisy peaks
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
        if max(speed) >= draw_speed_max or max(speed) < draw_speed_max*0.1:
            draw_speed_max=max(speed)*1.05
        ax1.axis(ymin=0,ymax=draw_speed_max)
        if draw_error_max is None:
            draw_error_max=max(error)*1.05
        if max(error) >= draw_error_max or max(error) < draw_error_max*0.1:
            draw_error_max=max(error)*1.05
        ax2.axis(ymin=0,ymax=draw_error_max)
        ax1.set_xlabel('lambda (mm)')
        ax1.set_ylabel('Speed/lamdot (mm/s)', color='g')
        ax2.set_ylabel('Error/Normal Error (mm/deg)', color='b')
        plt.title("Speed and Error Plot")
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
            df=DataFrame({'primitives':primitives1,'points':p_bp1,'q_bp':q_bp1})
            df.to_csv(ilc_output+'command_arm1_'+str(i)+'.csv',header=True,index=False)
            df=DataFrame({'primitives':primitives2,'points':p_bp2,'q_bp':q_bp2})
            df.to_csv(ilc_output+'command_arm2_'+str(i)+'.csv',header=True,index=False)
            DataFrame(curve_exe_js1).to_csv(ilc_output+'curve_js1_exe.csv',header=False,index=False)
            DataFrame(curve_exe_js2).to_csv(ilc_output+'curve_js2_exe.csv',header=False,index=False)
            np.save(ilc_output+'final_speed.npy',speed)
            np.save(ilc_output+'final_error.npy',error)
            np.save(ilc_output+'final_ang_error.npy',angle_error)

        ###########################plot for verification###################################
        # p_bp_relative,_=ms.form_relative_path(np.squeeze(q_bp1),np.squeeze(q_bp2),base2_R,base2_p)

        if max(error)>max_error and error_localmin_flag:
            print("Can't decrease anymore")
            break

        if max(error)>max_error:
            print("Use grad")
            error_localmin_flag=True
            use_grad=True
        
        if max(error)<max_error:
            error_localmin_flag=False

        max_error=max(error)

        if max(error) < max_error_tolerance and max(angle_error)<max_ang_error_tolerance and (std_speed/ave_speed*100)<velstd_tol:
            print("Tolerance Satisfied")
            break
    
        if max(error) < max_error_tolerance and max(angle_error)<max_ang_error_tolerance:
            print("Speed Tolerance Not Satisfied")
            break

        if use_grad:
            ##########################################calculate gradient for peaks######################################
            ###restore trajectory from primitives
            curve_interp1, curve_R_interp1, curve_js_interp1, breakpoints_blended=form_traj_from_bp(q_bp1,primitives1,robot1)
            curve_interp2, curve_R_interp2, curve_js_interp2, breakpoints_blended=form_traj_from_bp(q_bp2,primitives2,robot2)
            curve_js_blended1,curve_blended1,curve_R_blended1=blend_js_from_primitive(curve_interp1, curve_js_interp1, breakpoints_blended, primitives1,robot1,zone=10)
            curve_js_blended2,curve_blended2,curve_R_blended2=blend_js_from_primitive(curve_interp2, curve_js_interp2, breakpoints_blended, primitives2,robot2,zone=10)

            ###establish relative trajectory from blended trajectory
            _,_,_,_,relative_path_blended,relative_path_blended_R=form_relative_path(curve_js_blended1,curve_js_blended2,robot1,robot2)

            ###create copy to modify each peak individually
            p_bp1_new=copy.deepcopy(p_bp1)
            q_bp1_new=copy.deepcopy(q_bp1)
            p_bp2_new=copy.deepcopy(p_bp2)
            q_bp2_new=copy.deepcopy(q_bp2)
            for peak in peaks:
                ######gradient calculation related to nearest 3 points from primitive blended trajectory, not actual one
                _,peak_error_curve_idx=calc_error(relative_path_exe[peak],relative_path[:,:3])  # index of original curve closest to max error point

                ###get closest to worst case point on blended trajectory
                _,peak_error_curve_blended_idx=calc_error(relative_path_exe[peak],relative_path_blended)

                ###############get numerical gradient#####
                ###find closest 3 breakpoints
                order=np.argsort(np.abs(breakpoints_blended-peak_error_curve_blended_idx))
                breakpoint_interp_2tweak_indices=order[:3]

                de_dp=ilc.get_gradient_from_model_xyz_dual(\
                    [p_bp1,p_bp2],[q_bp1,q_bp2],breakpoints_blended,[curve_blended1,curve_blended2],peak_error_curve_blended_idx,[curve_exe_js1[peak],curve_exe_js2[peak]],relative_path[peak_error_curve_idx,:3],breakpoint_interp_2tweak_indices)


                p_bp1_new, q_bp1_new,p_bp2_new,q_bp2_new=ilc.update_bp_xyz_dual([p_bp1_new,p_bp2_new],[q_bp1_new,q_bp2_new],de_dp,error[peak],breakpoint_interp_2tweak_indices)

        else:
            ##########################################move towards error direction######################################
            error_bps_v1,error_bps_w1,error_bps_v2,error_bps_w2=ilc.get_error_direction_dual(relative_path,p_bp1,q_bp1,p_bp2,q_bp2,relative_path_exe,relative_path_exe_R,curve_exe1,curve_exe_R1,curve_exe2,curve_exe_R2)
            # error_bps_w1=np.zeros(error_bps_w1.shape)
            error_bps_w2=np.zeros(error_bps_w2.shape)
            # error_bps_v1=np.zeros(error_bps_v1.shape)
            # error_bps_v2=np.zeros(error_bps_v2.shape)

            p_bp1_new, q_bp1_new, p_bp2_new, q_bp2_new=ilc.update_error_direction_dual(relative_path,p_bp1,q_bp1,p_bp2,q_bp2,error_bps_v1,error_bps_w1,error_bps_v2,error_bps_w2)

        p_bp1_prev=copy.deepcopy(p_bp1)
        q_bp1_prev=copy.deepcopy(q_bp1)
        p_bp2_prev=copy.deepcopy(p_bp2)
        q_bp2_prev=copy.deepcopy(q_bp2)
        ###update
        p_bp1=p_bp1_new
        q_bp1=q_bp1_new
        p_bp2=p_bp2_new
        q_bp2=q_bp2_new
    
    return curve_exe_js1,curve_exe_js2,speed,error,np.rad2deg(angle_error),breakpoints1,primitives1,q_bp1,p_bp1,primitives2,q_bp2,p_bp2
from .ilc_toolbox import *
from pathlib import Path
import traceback

def motion_program_update(filepath,robot,robot_ip,robotMotionSend,vel,desired_curve_filename,desired_curvejs_filename,\
    err_tol,angerr_tol,velstd_tol,extstart,extend):
    try:
        curve = read_csv(desired_curve_filename,header=None).values
        curve=np.array(curve)
        curve_js = read_csv(desired_curvejs_filename,header=None).values
        curve_js=np.array(curve_js)

        # return error_descent_fanuc(filepath,robot,robot_ip,robotMotionSend,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls',extstart=extstart,extend=extend)
        if 'ABB' in robot.def_path:
            return error_descent_abb(filepath,robot,robotMotionSend,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls')
        if 'FANUC' in robot.def_path:
            return error_descent_fanuc(filepath,robot,robot_ip,robotMotionSend,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls',extstart=extstart,extend=extend)
    except:
        traceback.print_exc()

def error_descent_abb(filepath,robot,robotMotionSend,velocity,desired_curve,desired_curve_js,\
    error_tol=0.5,angerror_tol=3,velstd_tol=5,iteration_max=100,save_all_file=False,save_ls=False,save_name=''):

    curve=desired_curve
    curve_js=desired_curve_js

    ilc_output=filepath+'/result_speed_'+str(velocity)+'/'
    Path(ilc_output).mkdir(exist_ok=True)

    ms = robotMotionSend()

    breakpoints,primitives,p_bp,q_bp=ms.extract_data_from_cmd(filepath+'/command.csv')


    alpha = 0.5 # for gradient descent
    alpha_error_dir = 0.8 # for pushing in error direction

    ### speed,zone
    s = speeddata(velocity,9999999,9999999,999999)
    zone=10
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
        # ax1.legend(loc=0)
        # ax2.legend(loc=0)
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax1.legend(h1+h2, l1+l2, loc=1)

        #### save all things
        if save_all_file:
            # save fig
            plt.legend()
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
    error_tol=0.5,angerror_tol=3,velstd_tol=5,iteration_max=100,save_all_file=False,save_ls=False,save_name='',extstart=100,extend=100):

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
        
        ###execute,curve_fit_js only used for orientation
        logged_data=ms.exec_motions(robot,primitives,breakpoints,p_bp,q_bp,s,z)

        # print(logged_data)
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
from .baseline import *
from .constraint_solver import *

import numpy as np
from fanuc_motion_program_exec_client import *

def redundancy_resolution_baseline(filename, robot, exclude_extreme=True):

    curve = np.loadtxt(filename,delimiter=',')

    total_len=len(curve)
    
    H = pose_opt(robot,curve[:,:3],curve[:,3:])

    rot_in_j1=np.radians(-30)
    H[:3,3]=np.matmul(rot([0,0,1],rot_in_j1),H[:3,3])
    # H[:3,0]+=50
    H[:3,:3]=np.matmul(rot([0,0,1],rot_in_j1),H[:3,:3])

    curve_base,curve_normal_base=curve_frame_conversion(curve[:,:3],curve[:,3:],H)
    
    curve_js=[]
    for i in range(4):
        if i==0: # default
            print("x-axis as tangent vector")
            curve_js_all=find_js(robot,curve_base,curve_normal_base,False,False,False)
        elif i==1: # reverse x-axis as tangent vec
            print("reversed x-axis as tangent vector")
            curve_js_all=find_js(robot,curve_base,curve_normal_base,False,False,True)
        elif i==2: # y-axis as tangent vec
            print("y-axis as tangent vector")
            curve_js_all=find_js(robot,curve_base,curve_normal_base,False,True,False)
        else: # reverse y-axis as tangent vec
            print("reversed y-axis as tangent vector")
            curve_js_all=find_js(robot,curve_base,curve_normal_base,False,True,True)

        if exclude_extreme:
            curve_js_all_exclude=[]
            for js in curve_js_all:
                # use third joint to see if extreme
                if js[0][2]<np.pi/2:
                    curve_js_all_exclude.append(js)
            curve_js_all=curve_js_all_exclude

        if len(curve_js_all) > 0:
            J_min=[]
            for i in range(len(curve_js_all)):
                J_min.append(find_j_min(robot,curve_js_all[i]))

            J_min=np.array(J_min)
            curve_js=curve_js_all[np.argmin(J_min.min(axis=1))]
            break

    if len(curve_js)==0:
        print("Us QP")
        curve_js=redundancy_resolution_baseline_qp(robot,curve_base,curve_normal_base,exclude_extreme)

    return curve_base,curve_normal_base,curve_js,H

def redundancy_resolution_baseline_qp(robot,curve_base,curve_normal_base,exclude_extreme):

    opt=lambda_opt(curve_base,curve_normal_base,robot1=robot,steps=50000,v_cmd=500)

    ###get R first 
    curve_R=[]
    for i in range(len(curve_base)-1):
        # R_curve=direction2R(curve_normal[i],-curve[i+1]+curve[i])	
        R_curve=direction2R_Y(curve_normal_base[i],curve_base[i+1]-curve_base[i])	
        curve_R.append(R_curve)

    ###insert initial orientation
    curve_R.insert(0,curve_R[0])

    all_qinits = []
    for i in range(4):
        if i==0: # default
            curve_R_init = R_curve=direction2R(curve_normal_base[i],-curve_base[i+1]+curve_base[i])
        elif i==1: # reverse x-axis as tangent vec
            curve_R_init = R_curve=direction2R(curve_normal_base[i],curve_base[i+1]-curve_base[i])
        elif i==2: # y-axis as tangent vec
            curve_R_init = R_curve=direction2R_Y(curve_normal_base[i],-curve_base[i+1]+curve_base[i])
        else: # reverse y-axis as tangent vec
            curve_R_init = R_curve=direction2R_Y(curve_normal_base[i],curve_base[i+1]-curve_base[i])
        
        q_inits=np.array(robot.inv(curve_base[0],curve_R_init))
        all_qinits.extend(list(q_inits))
    
    q_init_norm = np.linalg.norm(all_qinits,axis=1)
    q_init = all_qinits[np.argsort(q_init_norm)[0]]

    try:
        print(np.degrees(q_init))
        curve_js=opt.single_arm_stepwise_optimize(q_init,curve_base,curve_normal_base)
    except:
        curve_js=[]

    return curve_js

def redundancy_resolution_diffevo(filename, baseline_pose_filename, curve_js_init_filename, robot, v_cmd=1000, full_opt=True, d_bounds=[], de_max_iter=500, de_time_limit=None):
    
    curve = np.loadtxt(filename,delimiter=',')
    curve_js_init = np.loadtxt(curve_js_init_filename,delimiter=',')

    # opt=lambda_opt(curve[:,:3],curve[:,3:],robot1=robot,steps=500,v_cmd=v_cmd)
    opt=lambda_opt(curve[:,:3],curve[:,3:],robot1=robot,steps=400,v_cmd=v_cmd)

    #read in initial curve pose
    curve_pose=np.loadtxt(baseline_pose_filename,delimiter=',')

    k,theta=R2rot(curve_pose[:3,:3])

    ###path constraints, position constraint and curve normal constraint
    lower_limit=np.array([-2*np.pi,-2*np.pi,-2*np.pi,0,-3000,0,-np.pi])
    upper_limit=np.array([2*np.pi,2*np.pi,2*np.pi,3000,3000,3000,np.pi])
    curve_base_init_0=np.dot(curve_pose[:3,:3],curve[0,:3]).T+curve_pose[:3,-1]
    curve_base_init_1=np.dot(curve_pose[:3,:3],curve[1,:3]).T+curve_pose[:3,-1]
    curve_normal_base_init=np.dot(curve_pose[:3,:3],curve[0,3:])
    q_init = curve_js_init[0]
    T_init = robot.fwd(q_init)
    R_temp=direction2R(curve_normal_base_init,-curve_base_init_1+curve_base_init_0)
    Rz_theta1 = np.matmul(R_temp.T,T_init.R)
    k_dum,theta1 = R2rot(Rz_theta1)

    if full_opt:    
        bnds=tuple(zip(lower_limit,upper_limit))
        x0 = np.hstack((k*theta,curve_pose[:-1,-1],[theta1]))
    else:
        assert len(d_bounds)==7, f"The length of the bounds differences should be 7"
        x0 = np.hstack((k*theta,curve_pose[:-1,-1],theta1))
        lower_limit = np.clip(x0-d_bounds,lower_limit,None)
        upper_limit = np.clip(x0+d_bounds,None,upper_limit)
        bnds=tuple(zip(lower_limit,upper_limit))

    print("Sanity Check")
    print(opt.curve_pose_opt2(x0))
    print("Sanity Check Done")

    st = time.time()

    def de_cb_timer(xk, convergence):
        print('cb:',convergence)
        print("xk:",xk)
        if de_time_limit:
            if time.time()-st>de_time_limit:
                print("DE over time limit")
                return True

    res = differential_evolution(opt.curve_pose_opt2, bnds, args=None,workers=-1,
                                    x0 = x0,
                                    strategy='best1bin', maxiter=de_max_iter,
                                    popsize=15, tol=1e-10,
                                    mutation=(0.5, 1), recombination=0.7,
                                    seed=None, 
                                    callback=de_cb_timer, 
                                    disp=False,
                                    polish=False, init='latinhypercube',
                                    atol=0.)
    
    theta0=np.linalg.norm(res.x[:3])
    k=res.x[:3]/theta0
    p_curve=res.x[3:-1]
    theta1=res.x[-1]

    R_curve=rot(k,theta0)
    H=H_from_RT(R_curve,p_curve)


    ###get initial q
    curve_base=np.dot(R_curve,opt.curve.T).T+np.tile(p_curve,(len(opt.curve),1))
    curve_normal_base=np.dot(R_curve,opt.curve_normal.T).T

    R_temp=direction2R(curve_normal_base[0],-curve_base[1]+curve_base[0])
    R=np.dot(R_temp,Rz(theta1))
    q_init=robot.inv(curve_base[0],R)[0]

    #########################################restore only given points, saves time##########################################################
    curve_js=opt.single_arm_stepwise_optimize(q_init,curve_base,curve_normal_base)

    return curve_base,curve_normal_base,curve_js,H

def redundancy_resolution_diffevo_dual(filename, base_T, robot1, robot2, q_init2_init, v_cmd=500, optimize_base=True, \
                                       full_opt=True, d_bounds=[], de_max_iter=500, de_time_limit=None):
    
    relative_path=read_csv(filename,header=None).values
    
    base2_R=base_T[:3,:3]
    base2_p=base_T[:-1,-1]
    base2_k,base2_theta=R2rot(base2_R)
    robot2.base_H=H_from_RT(base2_R,base2_p)

    opt=lambda_opt(relative_path[:,:3],relative_path[:,3:],robot1=robot1,robot2=robot2,steps=500,v_cmd=v_cmd)

    rot_init=0

    x_init = np.append(q_init2_init,base2_p[:2])
    x_init = np.append(x_init,base2_theta)
    x_init = np.append(x_init,rot_init)
    # print(input_x)
    print("Sanity Check")
    print(opt.dual_arm_opt_w_pose_3dof(x_init))
    print("Sanity Check Done")

    if optimize_base:
        lower_limit=np.hstack((robot2.lower_limit,[0,-2000],[-np.pi],[-np.pi]))
        upper_limit=np.hstack((robot2.upper_limit,[2000,2000],[np.pi],[np.pi]))
        bnds=tuple(zip(lower_limit,upper_limit))
        res = differential_evolution(opt.dual_arm_opt_w_pose_3dof, bnds, args=None,workers=-1,
                                        x0 = x_init,
                                        strategy='best1bin', maxiter=700,
                                        popsize=15, tol=1e-10,
                                        mutation=(0.5, 1), recombination=0.7,
                                        seed=None, callback=None, disp=True,
                                        polish=True, init='latinhypercube',
                                        atol=0)
        print(res)
        q_init2=res.x[:6]
        base2_p=np.array([res.x[6],res.x[7],base2_p[2]])		###fixed z height
        base2_theta=res.x[8]
        base2_R=Rz(base2_theta)
        rot_init=res.x[-1]

    else:
        lower_limit=np.hstack((robot2.lower_limit,[-np.pi]))
        upper_limit=np.hstack((robot2.upper_limit,[np.pi]))
        bnds=tuple(zip(lower_limit,upper_limit))
        res = differential_evolution(opt.dual_arm_opt_w_q2init, bnds, args=None,workers=-1,
                                        x0 = np.hstack((q_init2_init,[rot_init])),
                                        strategy='best1bin', maxiter=700,
                                        popsize=15, tol=1e-10,
                                        mutation=(0.5, 1), recombination=0.7,
                                        seed=None, callback=None, disp=True,
                                        polish=True, init='latinhypercube',
                                        atol=0)
    
        print(res)
        q_init2=res.x[:6]
        rot_init=res.x[-1]
    
    ## robot2 base
    robot2.base_H=H_from_RT(base2_R,base2_p)
    pose2_world_now=robot2.fwd(q_init2,world=True)

    ## init rotation in world frame
    R_temp=direction2R(pose2_world_now.R@opt.curve_normal[0],pose2_world_now.R@(-opt.curve[1]+opt.curve[0]))
    R=np.dot(R_temp,Rz(rot_init))

    q_init1=robot1.inv(np.matmul(pose2_world_now.R,opt.curve[0])+pose2_world_now.p,R)[0]

    opt=lambda_opt(relative_path[:,:3],relative_path[:,3:],robot1=robot1,robot2=robot2,steps=50000)
    q_out1,q_out2,j_out1,j_out2=opt.dual_arm_stepwise_optimize(q_init1,q_init2,w1=0.01,w2=0.02)

    jac_check_count=500
    jminall1=[]
    jminall2=[]
    for J in j_out1[::jac_check_count]:
        _,sv,_=np.linalg.svd(J)
        jminall1.append(sv)
    for J in j_out2[::jac_check_count]:
        _,sv,_=np.linalg.svd(J)
        jminall2.append(sv)
    print("J1 min svd:",np.min(jminall1))
    print("J2 min svd:",np.min(jminall2))

    base_T=np.eye(4)
    base_T[:-1,-1]=base2_p
    base_T[:3,:3]=base2_R

    return q_out1,q_out2,base_T
from .baseline import *
from .constraint_solver import *

import numpy as np
from fanuc_motion_program_exec_client import *

def redundancy_resolution_baseline(filename, robot):

    ## fanuc has different joint rotation axis
    if 'FANUC' in robot.robot_name:
        robot.j_compensation=np.array([1,1,1,1,1,1])

    curve = np.loadtxt(filename,delimiter=',')
    H = pose_opt(robot,curve[:,:3],curve[:,3:])
    curve_base,curve_normal_base=curve_frame_conversion(curve[:,:3],curve[:,3:],H)
    
    curve_js_all=find_js(robot,curve_base,curve_normal_base)

    if len(curve_js_all) > 0:
        J_min=[]
        for i in range(len(curve_js_all)):
            J_min.append(find_j_min(robot,curve_js_all[i]))

        J_min=np.array(J_min)
        curve_js=curve_js_all[np.argmin(J_min.min(axis=1))]
    else:
        curve_js=[]

    curve_js=[]
    if len(curve_js)==0:
        print("Us QP")
        curve_js=redundancy_resolution_baseline_qp(robot,curve_base,curve_normal_base)

    if 'FANUC' in robot.robot_name:
        curve_js[:,2:]=-1*curve_js[:,2:]
        robot.j_compensation=np.array([1,1,-1,-1,-1,-1])

    return curve_base,curve_normal_base,curve_js,H

def redundancy_resolution_baseline_qp(robot,curve_base,curve_normal_base):

    opt=lambda_opt(curve_base,curve_normal_base,robot1=robot,steps=50000,v_cmd=500)

    ###get R first 
    curve_R=[]
    for i in range(len(curve_base)-1):
        # R_curve=direction2R(curve_normal[i],-curve[i+1]+curve[i])	
        R_curve=direction2R_Y(curve_normal_base[i],curve_base[i+1]-curve_base[i])	
        curve_R.append(R_curve)

    ###insert initial orientation
    curve_R.insert(0,curve_R[0])

    ###get all possible initial config
    try:
        q_inits=np.array(robot.inv(curve_base[0],curve_R[0]))
        q1_candidate=[]
        for q in q_inits:
            if q[2]<=np.pi/2:
                q1_candidate.append(q)
        if len(q1_candidate)>0:
            q1_4_dist=9999
            for q in q1_candidate:
                if np.fabs(q[3])<q1_4_dist:
                    q1_4_dist=np.fabs(q[3])
                    q_init=copy.deepcopy(q)
        else:
            q1_4_dist=9999
            for q in q_inits:
                if np.fabs(q[3])<q1_4_dist:
                    q1_4_dist=np.fabs(q[3])
                    q_init=copy.deepcopy(q)
    except:
        traceback.print_exc()
        print('no solution available')
        return

    print(np.degrees(q_init))
    try:
        curve_js=opt.single_arm_stepwise_optimize(q_init,curve_base,curve_normal_base)
    except:
        curve_js=[]

    return curve_js

def redundancy_resolution_diffevo(filename, baseline_pose_filename, robot, v_cmd=1000):
    print(baseline_pose_filename)
    curve = np.loadtxt(filename,delimiter=',')

    ## fanuc has different joint rotation axis
    if 'FANUC' in robot.robot_name:
        robot.j_compensation=np.array([1,1,1,1,1,1])

    opt=lambda_opt(curve[:,:3],curve[:,3:],robot1=robot,steps=500,v_cmd=v_cmd)

    #read in initial curve pose
    curve_pose=np.loadtxt(baseline_pose_filename,delimiter=',')

    k,theta=R2rot(curve_pose[:3,:3])

    ###path constraints, position constraint and curve normal constraint
    lowerer_limit=np.array([-2*np.pi,-2*np.pi,-2*np.pi,0,-3000,0,-np.pi])
    upper_limit=np.array([2*np.pi,2*np.pi,2*np.pi,3000,3000,3000,np.pi])
    bnds=tuple(zip(lowerer_limit,upper_limit))


    res = differential_evolution(opt.curve_pose_opt2, bnds, args=None,workers=-1,
                                    x0 = np.hstack((k*theta,curve_pose[:-1,-1],[0])),
                                    strategy='best1bin', maxiter=500,
                                    popsize=15, tol=1e-10,
                                    mutation=(0.5, 1), recombination=0.7,
                                    seed=None, callback=None, disp=False,
                                    polish=True, init='latinhypercube',
                                    atol=0.)

    theta0=np.linalg.norm(res.x[:3])
    k=res.x[:3]/theta0
    p_curve=res.x[3:-1]
    theta1=res.x[-1]

    R_curve=rot(k,theta0)
    H=Rp2H(p_curve,R_curve)


    ###get initial q
    curve_base=np.dot(R_curve,opt.curve.T).T+np.tile(p_curve,(len(opt.curve),1))
    curve_normal_base=np.dot(R_curve,opt.curve_normal.T).T

    R_temp=direction2R(curve_normal_base[0],-curve_base[1]+curve_base[0])
    R=np.dot(R_temp,Rz(theta1))
    q_init=robot.inv(curve_base[0],R)[0]

    #########################################restore only given points, saves time##########################################################
    curve_js=opt.single_arm_stepwise_optimize(q_init,curve_base,curve_normal_base)

    if 'FANUC' in robot.robot_name:
        curve_js[:,2:]=-1*curve_js[:,2:]
        robot.j_compensation=np.array([1,1,-1,-1,-1,-1])

    return curve_base,curve_normal_base,curve_js,H

def redundancy_resolution_diffevo_dual(filename, base_T, robot1, robot2, q_init2_init, v_cmd=500, optimize_base=True):

    if 'FANUC' in robot1.robot_name:
        q_init2_init[2:]=-1*q_init2_init[2:]
        robot1.j_compensation=np.array([1,1,1,1,1,1])
        robot2.j_compensation=np.array([1,1,1,1,1,1])
    
    relative_path=read_csv(filename,header=None).values
    
    base2_R=base_T[:3,:3]
    base2_p=base_T[:-1,-1]
    base2_k,base2_theta=R2rot(base2_R)
    robot2.base_H=H_from_RT(base2_R,base2_p)

    opt=lambda_opt(relative_path[:,:3],relative_path[:,3:],robot1=robot1,robot2=robot2,steps=500,v_cmd=v_cmd)

    rot_init=0

    input_x = np.append(q_init2_init,base2_p[:2])
    input_x = np.append(input_x,base2_theta)
    input_x = np.append(input_x,rot_init)
    # print(input_x)
    print("Sanity Check")
    print(opt.dual_arm_opt_w_pose_3dof(input_x))
    print("Sanity Check Done")

    if optimize_base:
        lower_limit=np.hstack((robot2.lower_limit,[0,-2000],[-np.pi],[-np.pi]))
        upper_limit=np.hstack((robot2.upper_limit,[2000,2000],[np.pi],[np.pi]))
        bnds=tuple(zip(lower_limit,upper_limit))
        res = differential_evolution(opt.dual_arm_opt_w_pose_3dof, bnds, args=None,workers=-1,
                                        x0 = np.hstack((q_init2_init,base2_p[0],base2_p[1],base2_theta,[rot_init])),
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

    if 'FANUC' in robot1.robot_name:
        robot1.j_compensation=np.array([1,1,-1,-1,-1,-1])
        robot2.j_compensation=np.array([1,1,-1,-1,-1,-1])
        q_out1[:,2:]=-1*q_out1[:,2:]
        q_out2[:,2:]=-1*q_out2[:,2:]

    return q_out1,q_out2,base_T
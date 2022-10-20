from .baseline import *
from .constraint_solver import *

import numpy as np

def redundancy_resolution_baseline(filename, robot):

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

    return curve_base,curve_normal_base,curve_js,H

def redundancy_resolution_diffevo(filename, baseline_pose_filename, robot):
    curve = np.loadtxt(filename,delimiter=',')


    v_cmd=1555
    opt=lambda_opt(curve_dense[:,:3],curve_dense[:,3:],robot1=robot,steps=500,v_cmd=v_cmd)

    #read in initial curve pose
    curve_pose=np.loadtxt(baseline_pose_filename,delimiter=',')

    k,theta=R2rot(curve_pose[:3,:3])

    ###path constraints, position constraint and curve normal constraint
    lowerer_limit=np.array([-2*np.pi,-2*np.pi,-2*np.pi,0,-3000,0,-np.pi])
    upper_limit=np.array([2*np.pi,2*np.pi,2*np.pi,3000,3000,3000,np.pi])
    bnds=tuple(zip(lowerer_limit,upper_limit))


    res = differential_evolution(opt.curve_pose_opt2, bnds, args=None,workers=11,
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

    return curve_base,curve_normal_base,curve_js,H
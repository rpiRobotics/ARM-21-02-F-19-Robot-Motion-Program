from math import radians,pi
import numpy as np
from pandas import *
import yaml
from matplotlib import pyplot as plt

from general_robotics_toolbox import *
from general_robotics_toolbox.general_robotics_toolbox_invkin import *
import sys

from baseline.baseline_m710ic_part1 import *
from baseline.baseline_m710ic_part2 import *
from ILC.max_gradient_fanuc import *
sys.path.append('ILC')

def redundanct_resolution(filename, robot):

    curve = read_csv(filename,header=None).values
    H = pose_opt(robot,curve[:,:3],curve[:,3:])
    curve_base,curve_normal_base=curve_frame_conversion(curve[:,:3],curve[:,3:],H)
    curve_js_all=find_js(robot,curve_base,curve_normal_base)
    J_min=[]
    for i in range(len(curve_js_all)):
        J_min.append(find_j_min(robot,curve_js_all[i]))

    J_min=np.array(J_min)
    curve_js=curve_js_all[np.argmin(J_min.min(axis=1))]

    return curve_base,curve_normal_base,curve_js,H

def motion_program_generation(filename,robot,total_seg):
    
    curve_js = read_csv(filename,header=None).values
    curve_js=np.array(curve_js)

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

    return breakpoints,primitives,q_bp,p_bp

def motion_program_update(filepath,robot,vel,desired_curve_filename,desired_curvejs_filename,\
    err_tol,angerr_tol,velstd_tol):
    
    curve = read_csv(desired_curve_filename,header=None).values
    curve=np.array(curve)
    curve_js = read_csv(desired_curvejs_filename,header=None).values
    curve_js=np.array(curve_js)

    return max_grad_descent(filepath,robot,vel,curve,curve_js,err_tol,angerr_tol,velstd_tol,save_all_file=True,save_ls=True,save_name='final_ls')


import numpy as np
import sys, traceback, os
from .greedy import *

def motion_program_generation_baseline(filename,robot,total_seg):
    ###generate motion primitive command for equally spaced moveL as baseline
    curve_js =np.loadtxt(filename,delimiter=',')
    breakpoints=np.linspace(0,len(curve_js)-1,total_seg+1).astype(int)
    primitives = ['moveabsj_fit']+['movel_fit']*total_seg

    q_bp=curve_js[breakpoints]
    p_bp=robot.fwd(q_bp).p_all

    # q_bp.reshape(len(breakpoints),1,len(curve_js[0]))
    q_bp=q_bp.tolist()
    p_bp=p_bp.tolist()
    for i in range(len(q_bp)):
        q_bp[i]=[np.array(q_bp[i])]
        p_bp[i]=[np.array(p_bp[i])]

    return breakpoints,primitives,q_bp,p_bp

def motion_program_generation_greedy(filename,robot,greedy_thresh):
    ###generate motion primitive command for with greedy algorithm
    curve_js =np.loadtxt(filename,delimiter=',')

    min_length=10
    greedy_fit_obj=greedy_fit(robot,curve_js, min_length=min_length,max_error_threshold=greedy_thresh)

    breakpoints,primitives,p_bp,q_bp=greedy_fit_obj.fit_under_error()
    
    ############insert initial configuration#################
    primitives.insert(0,'moveabsj_fit')
    p_bp.insert(0,[greedy_fit_obj.curve_fit[0]])
    q_bp.insert(0,[greedy_fit_obj.curve_fit_js[0]])

    ###adjust breakpoint index
    breakpoints[1:]=breakpoints[1:]-1

    print(len(breakpoints))
    print(len(primitives))
    print(len(p_bp))
    print(len(q_bp))

    return breakpoints,primitives,q_bp,p_bp
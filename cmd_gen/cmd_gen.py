import numpy as np
import sys, traceback, os
from .greedy import *

def motion_program_generation_baseline(filename,robot,total_seg):
    ###generate motion primitive command for equally spaced moveL as baseline
    curve_js =np.loadtxt(filename,delimiter=',')
    breakpoints=np.linspace(0,len(curve_js)-1,total_seg+1).astype(int)
    
    if 'FANUC' in robot.robot_name:
        primitives = ['movej_fit']+['movel_fit']*total_seg
    else:
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

def motion_program_generation_baseline(filename1,robot1,filename2,robot2,total_seg):
    ###generate motion primitive command for equally spaced moveL as baseline
    curve_js1 =np.loadtxt(filename1,delimiter=',')
    curve_js2 =np.loadtxt(filename2,delimiter=',')
    breakpoints=np.linspace(0,len(curve_js1)-1,total_seg+1).astype(int)
    if 'FANUC' in robot1.robot_name:
        primitives = ['movej_fit']+['movel_fit']*total_seg
    else:
        primitives = ['moveabsj_fit']+['movel_fit']*total_seg

    q_bp1=curve_js1[breakpoints]
    p_bp1=robot1.fwd(q_bp1).p_all
    q_bp2=curve_js2[breakpoints]
    p_bp2=robot2.fwd(q_bp2).p_all

    # q_bp.reshape(len(breakpoints),1,len(curve_js[0]))
    q_bp1=q_bp1.tolist()
    p_bp1=p_bp1.tolist()
    q_bp2=q_bp2.tolist()
    p_bp2=p_bp2.tolist()

    for i in range(len(q_bp1)):
        q_bp1[i]=[np.array(q_bp1[i])]
        p_bp1[i]=[np.array(p_bp1[i])]
        q_bp2[i]=[np.array(q_bp2[i])]
        p_bp2[i]=[np.array(p_bp2[i])]

    return breakpoints,primitives,q_bp1,p_bp1,breakpoints,primitives,q_bp2,p_bp2

def motion_program_generation_greedy(filename,robot,greedy_thresh):
    ###generate motion primitive command for with greedy algorithm
    curve_js =np.loadtxt(filename,delimiter=',')

    min_length=10
    greedy_fit_obj=greedy_fit(robot,curve_js, min_length=min_length,max_error_threshold=greedy_thresh)

    breakpoints,primitives,p_bp,q_bp=greedy_fit_obj.fit_under_error()
    
    ############insert initial configuration#################
    if 'FANUC' in robot.robot_name:
        primitives.insert(0,'movej_fit')
    else:
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

def motion_program_generation_greedy_dual(filename1,robot1,filename2,robot2,greedy_thresh):

    curve_js1 =np.loadtxt(filename1,delimiter=',')
    curve_js2 =np.loadtxt(filename2,delimiter=',')

    min_length=0
    greedy_fit_obj=greedy_fit(robot1,robot2,curve_js1[::1],curve_js2[::1],min_length,greedy_thresh)

    greedy_fit_obj.primitives={'movel_fit':greedy_fit_obj.movel_fit,'movec_fit':greedy_fit_obj.movec_fit}

    breakpoints,primitives_choices1,p_bp1,q_bp1,primitives_choices2,p_bp2,q_bp2=greedy_fit_obj.fit_under_error()

    breakpoints[1:]=breakpoints[1:]-1

    print(len(breakpoints))
    print(len(primitives_choices1))
    print(len(p_bp1))
    print(len(q_bp1))
    print(len(primitives_choices2))
    print(len(p_bp2))
    print(len(q_bp2))
    
    return breakpoints,primitives_choices1,q_bp1,p_bp1,breakpoints,primitives_choices2,q_bp2,p_bp2
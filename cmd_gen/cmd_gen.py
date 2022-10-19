import numpy as np
from pandas import *
import sys, traceback, os
from robots_def import *
from utils import *

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

def motion_program_generation_greedy(filename,robot,total_seg):
    pass
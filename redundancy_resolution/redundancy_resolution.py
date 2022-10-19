from .baseline import *
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

def redundancy_resolution_diffevo(filename, robot):
    pass
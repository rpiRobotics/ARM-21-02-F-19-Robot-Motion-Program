from math import radians,pi
import numpy as np
from pandas import *
import yaml
from matplotlib import pyplot as plt

from general_robotics_toolbox import *
from general_robotics_toolbox.general_robotics_toolbox_invkin import *
import sys


# from simulation.roboguide.fanuc_toolbox.fanuc_client import FANUCClient, TPMotionProgram, joint2robtarget, jointtarget, robtarget
# from toolbox.robots_def import arb_robot, m900ia
sys.path.append('toolbox')
from robots_def import *
from utils import *
from lambda_calc import *

def main():

    all_objtype=['wood','blade']
    # all_objtype=['blade']
    # all_objtype=['wood']

    # num_ls=[80,100,150]
    num_ls=[100]

    for obj_type in all_objtype:

        # obj_type='wood'
        # obj_type='blade'
        print(obj_type)
        
        data_dir='../data/baseline_m710ic/'+obj_type+'/'

        robot=m710ic(d=50)
        # curve = read_csv(data_dir+"Curve_in_base_frame.csv",header=None).values
        ###read actual curve
        curve_js = read_csv(data_dir+"Curve_js.csv",header=None).values
        curve_js=np.array(curve_js)

        for num_l in num_ls:
            breakpoints=np.linspace(0,len(curve_js),num_l+1).astype(int)
            breakpoints[1:]=breakpoints[1:]-1

            primitives_choices=['movej_fit']
            points=[curve_js[0]]
            for i in breakpoints[1:]:
                primitives_choices.append('movel_fit')
                points.append(curve_js[i])
            points=np.array(points)
            
            ## save commands
            df=DataFrame({'breakpoints':breakpoints,'primitives':primitives_choices,\
                'J1':points[:,0],'J2':points[:,1],\
                'J3':points[:,2],'J4':points[:,3],\
                'J5':points[:,4],'J6':points[:,5]})
            df.to_csv(data_dir+str(num_l)+'/command.csv')
            df=DataFrame()

if __name__ == '__main__':
    main()
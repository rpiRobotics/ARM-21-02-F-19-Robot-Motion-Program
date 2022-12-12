import numpy as np
from fanuc_motion_program_exec_client import *
from fanuc_utils import *
import sys
from io import StringIO

robot2_name='FANUC_lrmate200id'
robot2=robot_obj(robot2_name,'../config/'+robot2_name+'_robot_default_config.yml',tool_file_path='../data/curve_1/FANUC_m10ia_FANUC_lrmate200id/tcp_workpiece.csv',base_transformation_file='../data/curve_1/FANUC_m10ia_FANUC_lrmate200id/diffevo_qinit2/base.csv',acc_dict_path='../config/'+robot2_name+'_acc_compensate.pickle',j_compensation=[1,1,-1,-1,-1,-1])
##### ugly cheat
robot_flange=Transform(Ry(np.pi/2)@Rz(np.pi),[0,0,0])
robot_tcp=Transform(wpr2R([-89.895,84.408,-67.096]),[316.834,0.39,5.897])
robot_flange_tcp=robot_flange*robot_tcp
robot1=m10ia(R_tool=robot_flange_tcp.R,p_tool=robot_flange_tcp.p,d=0,acc_dict_path='../config/FANUC_m10ia_acc.pickle')
################

ms = MotionSendFANUC(group=1,uframe=1,utool=3,robot_ip='127.0.0.2',robot1=robot1,robot2=robot2,utool2=2)

breakpoints1,primitives1,p_bp1,q_bp1,_=ms.extract_data_from_cmd('../data/curve_1/FANUC_m10ia_FANUC_lrmate200id/diffevo_qinit2/50L_dual/command1.csv')
breakpoints2,primitives2,p_bp2,q_bp2,_=ms.extract_data_from_cmd('../data/curve_1/FANUC_m10ia_FANUC_lrmate200id/diffevo_qinit2/50L_dual/command2.csv')
p_bp1,q_bp1,p_bp2,q_bp2=ms.extend_dual(ms.robot1,p_bp1,q_bp1,primitives1,ms.robot2,p_bp2,q_bp2,primitives2,breakpoints1,0,extension_start=10,extension_end=10)  ## curve_1, greedy 0.2

###execution with plant
z=100
s=50
logged_data=ms.exec_motions_multimove_test(robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,s,z)
StringData=StringIO(logged_data.decode('utf-8'))
df = read_csv(StringData, sep =",")
print(df)
import numpy as np
from copy import deepcopy
from pandas import read_csv, DataFrame
from fanuc_motion_program_exec_client import *
from general_robotics_toolbox import *
import threading

from utils import *
from lambda_calc import *
from robots_def import *

class MotionSendFANUC(object):
    def __init__(self,group=1,uframe=1,utool=2,robot_ip='127.0.0.2',robot1=m710ic(d=50),robot2=m900ia(),group2=2,uframe2=1,utool2=2,robot_ip2=None) -> None:
        
        self.client = FANUCClient(robot_ip)
        if robot_ip2 is not None:
            self.client = FANUCClient(robot_ip=robot_ip,robot_ip2=robot_ip2,robot_user2='Robot2')
        
        # robot1 roboguide info
        self.group = group
        self.uframe = uframe
        self.utool = utool

        # robot2 roboguide info
        self.group2 = group2
        self.uframe2 = uframe2
        self.utool2 = utool2

        # robots
        self.robot1=robot1
        self.robot2=robot2
    
    def exec_motions(self,robot,primitives,breakpoints,p_bp,q_bp,speed,zone,client=None,group=None,uframe=None,utool=None):

        if client is None:
            client=self.client
        if group is None:
            group=self.group
        if uframe is None:
            uframe=self.uframe
        if utool is None:
            utool=self.utool

        tp_pre = TPMotionProgram()

        # move to start
        j0 = joint2robtarget(q_bp[0][0],robot,group,uframe,utool)
        tp_pre.moveJ(j0,50,'%',-1)
        tp_pre.moveJ(j0,5,'%',-1)
        client.execute_motion_program(tp_pre)

        #### for speed regulation
        if (type(speed) is int) or (type(speed) is float):
            all_speed=np.ones(len(primitives))*int(speed)
        else:
            assert len(speed) == len(primitives), "Speed list must have the same length as primitives"
            all_speed=np.array(speed)

        # start traj
        tp = TPMotionProgram()
        for i in range(1,len(primitives)):
            if i == len(primitives)-1:
                this_zone = -1
            else:
                this_zone = zone
            
            # speed
            this_speed=int(all_speed[i])

            if primitives[i]=='movel_fit':
                robt = joint2robtarget(q_bp[i][0],robot,group,uframe,utool)
                tp.moveL(robt,this_speed,'mmsec',this_zone)
            elif primitives[i]=='movec_fit':
                robt_mid = joint2robtarget(q_bp[i][0],robot,group,uframe,utool)
                robt = joint2robtarget(q_bp[i][1],robot,group,uframe,utool)
                tp.moveC(robt_mid,robt,this_speed,'mmsec',this_zone)
            else: #moveJ
                # robt = jointtarget(group,uframe,utool,np.degrees(q_bp[i][0]),[0]*6)
                robt = joint2robtarget(q_bp[i][0],robot,group,uframe,utool)
                # tp.moveJ(robt,this_speed,'%',this_zone)
                tp.moveJ(robt,this_speed,'msec',this_zone)
        return client.execute_motion_program(tp)

    def exec_motions_multimove(self,robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,speed,zone,coord=[]):

        tp_follow = TPMotionProgram(self.utool,self.uframe)
        tp_lead = TPMotionProgram(self.utool2,self.uframe2)
        #### move to start
        # robot1
        j0 = joint2robtarget(q_bp1[0][0],robot1,self.group,self.uframe,self.utool)
        tp_follow.moveJ(j0,50,'%',-1)
        tp_follow.moveJ(j0,5,'%',-1)
        # robot2
        j0 = joint2robtarget(q_bp2[0][0],robot2,self.group2,self.uframe2,self.utool2)
        tp_lead.moveJ(j0,50,'%',-1)
        tp_lead.moveJ(j0,5,'%',-1)
        self.client.execute_motion_program_coord(tp_lead,tp_follow)

        #### coordinated motion in the trajectory
        if len(coord) == 0:
            coord = np.ones(len(primitives1))
        assert len(coord) == len(primitives1) , "Coordination string must have the same length as primitives"

        #### for speed regulation
        if (type(speed) is int) or (type(speed) is float):
            all_speed=np.ones(len(primitives1))*int(speed)
        else:
            assert len(speed) == len(primitives1), "Speed list must have the same length as primitives"
            all_speed=np.array(speed)

        #### start traj
        tp_follow = TPMotionProgram(self.utool,self.uframe)
        tp_lead = TPMotionProgram(self.utool2,self.uframe2)
        for i in range(1,len(primitives1)):
            if i == len(primitives1)-1:
                # this_zone = zone
                this_zone = -1
            else:
                this_zone = zone
            option=''
            if coord[i]:
                option='COORD'
            # speed
            this_speed=int(all_speed[i])

            if primitives1[i]=='movel_fit':
                # robot1
                robt1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveL(robt1,this_speed,'mmsec',this_zone,option)
                # tp_follow.moveL(robt1,1,'msec',this_zone,option)
                # robot2
                robt2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveL(robt2,this_speed,'mmsec',this_zone,option)
                # tp_lead.moveL(robt2,1,'msec',this_zone,option)
            elif primitives1[i]=='movec_fit':
                # robot1
                robt_mid1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                robt1 = joint2robtarget(q_bp1[i][1],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveC(robt_mid1,robt1,this_speed,'mmsec',this_zone,option)
                # robot2
                robt_mid2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                robt2 = joint2robtarget(q_bp2[i][1],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveC(robt_mid2,robt2,this_speed,'mmsec',this_zone,option)
            else: #moveJ
                robt1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                # tp_follow.moveJ(robt1,this_speed,'%',this_zone)
                tp_follow.moveJ(robt1,this_speed,'msec',this_zone)
                robt2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                # tp_lead.moveJ(robt2,this_speed,'%',this_zone)
                tp_lead.moveJ(robt2,this_speed,'msec',this_zone)
        
        return self.client.execute_motion_program_coord(tp_lead,tp_follow)
    
    def exec_motions_multimove_nocoord(self,robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,speed1,speed2,zone1,zone2):

        tp_follow = TPMotionProgram(self.utool,self.uframe)
        tp_lead = TPMotionProgram(self.utool2,self.uframe2)
        #### move to start
        # robot1
        j0 = joint2robtarget(q_bp1[0][0],robot1,self.group,self.uframe,self.utool)
        tp_follow.moveJ(j0,50,'%',-1)
        tp_follow.moveJ(j0,5,'%',-1)
        # robot2
        j0 = joint2robtarget(q_bp2[0][0],robot2,self.group2,self.uframe2,self.utool2)
        tp_lead.moveJ(j0,50,'%',-1)
        tp_lead.moveJ(j0,5,'%',-1)
        self.client.execute_motion_program_multi(tp_follow,tp_lead)
        
        #### for speed1 regulation
        if (type(speed1) is int) or (type(speed1) is float):
            all_speed=np.ones(len(primitives1))*int(speed1)
        else:
            assert len(speed1) == len(primitives1), "Speed list must have the same length as primitives"
            all_speed=np.array(speed1)

        #### start traj, follower, robot1
        tp_follow = TPMotionProgram(self.utool,self.uframe)
        for i in range(1,len(primitives1)):
            if i == len(primitives1)-1:
                # this_zone = zone1
                this_zone = -1
            else:
                this_zone = zone1
            option=''
            # speed
            this_speed=int(all_speed[i])

            if primitives1[i]=='movel_fit':
                # robot1
                robt1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveL(robt1,this_speed,'mmsec',this_zone,option)
                # tp_follow.moveL(robt1,this_speed,'msec',this_zone,option)
            elif primitives1[i]=='movec_fit':
                # robot1
                robt_mid1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                robt1 = joint2robtarget(q_bp1[i][1],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveC(robt_mid1,robt1,this_speed,'mmsec',this_zone,option)
            else: #moveJ
                robt1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveJ(robt1,this_speed,'%',this_zone)
                # tp_follow.moveJ(robt1,this_speed,'msec',this_zone)
        
        #### for speed2 regulation
        if (type(speed2) is int) or (type(speed2) is float):
            all_speed=np.ones(len(primitives2))*int(speed2)
        else:
            assert len(speed2) == len(primitives2), "Speed list must have the same length as primitives"
            all_speed=np.array(speed2)
        
        #### start traj, leader, robot2
        tp_lead = TPMotionProgram(self.utool2,self.uframe2)
        for i in range(1,len(primitives2)):
            if i == len(primitives2)-1:
                # this_zone = zone2
                this_zone = -1
            else:
                this_zone = zone2
            option=''
            # speed
            this_speed=int(all_speed[i])

            if primitives2[i]=='movel_fit':
                # robot2
                robt2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveL(robt2,this_speed,'mmsec',this_zone,option)
                # tp_lead.moveL(robt2,this_speed,'msec',this_zone,option)
            elif primitives2[i]=='movec_fit':
                # robot2
                robt_mid2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                robt2 = joint2robtarget(q_bp2[i][1],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveC(robt_mid2,robt2,this_speed,'mmsec',this_zone,option)
            else: #moveJ
                robt2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveJ(robt2,this_speed,'%',this_zone)
                # tp_lead.moveJ(robt2,this_speed,'msec',this_zone)
        
        return self.client.execute_motion_program_multi(tp_follow,tp_lead)
    
    def exec_motions_multimove_separate(self,robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,speed1,speed2,zone1,zone2):
        
        tp_follow = TPMotionProgram(self.utool,self.uframe)
        tp_lead = TPMotionProgram(self.utool2,self.uframe2)
        #### move to start
        # robot1
        j0 = joint2robtarget(q_bp1[0][0],robot1,self.group,self.uframe,self.utool)
        tp_follow.moveJ(j0,50,'%',-1)
        tp_follow.moveJ(j0,5,'%',-1)
        # robot2
        j0 = joint2robtarget(q_bp2[0][0],robot2,self.group2,self.uframe2,self.utool2)
        tp_lead.moveJ(j0,50,'%',-1)
        tp_lead.moveJ(j0,5,'%',-1)
        self.client.execute_motion_program_thread(tp_follow,tp_lead)
        
        #### for speed1 regulation
        if (type(speed1) is int) or (type(speed1) is float):
            all_speed=np.ones(len(primitives1))*int(speed1)
        else:
            assert len(speed1) == len(primitives1), "Speed list must have the same length as primitives"
            all_speed=np.array(speed1)

        #### start traj, follower, robot1
        tp_follow = TPMotionProgram(self.utool,self.uframe)
        for i in range(1,len(primitives1)):
            if i == len(primitives1)-1:
                # this_zone = zone1
                this_zone = -1
            else:
                this_zone = zone1
            option=''
            # speed
            this_speed=int(all_speed[i])

            if primitives1[i]=='movel_fit':
                # robot1
                robt1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveL(robt1,this_speed,'mmsec',this_zone,option)
                # tp_follow.moveL(robt1,this_speed,'msec',this_zone,option)
            elif primitives1[i]=='movec_fit':
                # robot1
                robt_mid1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                robt1 = joint2robtarget(q_bp1[i][1],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveC(robt_mid1,robt1,this_speed,'mmsec',this_zone,option)
            else: #moveJ
                robt1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveJ(robt1,this_speed,'%',this_zone)
                # tp_follow.moveJ(robt1,this_speed,'msec',this_zone)
        
        #### for speed2 regulation
        if (type(speed2) is int) or (type(speed2) is float):
            all_speed=np.ones(len(primitives2))*int(speed2)
        else:
            assert len(speed2) == len(primitives2), "Speed list must have the same length as primitives"
            all_speed=np.array(speed2)
        
        #### start traj, leader, robot2
        tp_lead = TPMotionProgram(self.utool2,self.uframe2)
        for i in range(1,len(primitives2)):
            if i == len(primitives2)-1:
                # this_zone = zone2
                this_zone = -1
            else:
                this_zone = zone2
            option=''
            # speed
            this_speed=int(all_speed[i])

            if primitives2[i]=='movel_fit':
                # robot2
                robt2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveL(robt2,this_speed,'mmsec',this_zone,option)
                # tp_lead.moveL(robt2,this_speed,'msec',this_zone,option)
            elif primitives2[i]=='movec_fit':
                # robot2
                robt_mid2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                robt2 = joint2robtarget(q_bp2[i][1],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveC(robt_mid2,robt2,this_speed,'mmsec',this_zone,option)
            else: #moveJ
                robt2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveJ(robt2,this_speed,'%',this_zone)
                # tp_lead.moveJ(robt2,this_speed,'msec',this_zone)
        
        return self.client.execute_motion_program_thread(tp_follow,tp_lead)
    
    def exec_motions_multimove_separate2(self,robot1,robot2,primitives1,primitives2,p_bp1,p_bp2,q_bp1,q_bp2,speed1,speed2,zone1,zone2):
        
        tp_follow = TPMotionProgram(self.utool,self.uframe)
        tp_lead = TPMotionProgram(self.utool2,self.uframe2)
        #### move to start
        # robot1
        j0 = joint2robtarget(q_bp1[0][0],robot1,self.group,self.uframe,self.utool)
        tp_follow.moveJ(j0,50,'%',-1)
        tp_follow.moveJ(j0,5,'%',-1)
        # robot2
        j0 = joint2robtarget(q_bp2[0][0],robot2,self.group2,self.uframe2,self.utool2)
        tp_lead.moveJ(j0,50,'%',-1)
        tp_lead.moveJ(j0,5,'%',-1)
        self.client.execute_motion_program_connect(tp_follow,tp_lead)
        
        #### for speed1 regulation
        if (type(speed1) is int) or (type(speed1) is float):
            all_speed=np.ones(len(primitives1))*int(speed1)
        else:
            assert len(speed1) == len(primitives1), "Speed list must have the same length as primitives"
            all_speed=np.array(speed1)

        #### start traj, follower, robot1
        tp_follow = TPMotionProgram(self.utool,self.uframe)
        for i in range(1,len(primitives1)):
            if i == len(primitives1)-1:
                # this_zone = zone1
                this_zone = -1
            else:
                this_zone = zone1
            option=''
            # speed
            this_speed=int(all_speed[i])

            if primitives1[i]=='movel_fit':
                # robot1
                robt1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveL(robt1,this_speed,'mmsec',this_zone,option)
                # tp_follow.moveL(robt1,this_speed,'msec',this_zone,option)
            elif primitives1[i]=='movec_fit':
                # robot1
                robt_mid1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                robt1 = joint2robtarget(q_bp1[i][1],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveC(robt_mid1,robt1,this_speed,'mmsec',this_zone,option)
            else: #moveJ
                robt1 = joint2robtarget(q_bp1[i][0],robot1,self.group,self.uframe,self.utool)
                tp_follow.moveJ(robt1,this_speed,'%',this_zone)
                # tp_follow.moveJ(robt1,this_speed,'msec',this_zone)
        
        #### for speed2 regulation
        if (type(speed2) is int) or (type(speed2) is float):
            all_speed=np.ones(len(primitives2))*int(speed2)
        else:
            assert len(speed2) == len(primitives2), "Speed list must have the same length as primitives"
            all_speed=np.array(speed2)
        
        #### start traj, leader, robot2
        tp_lead = TPMotionProgram(self.utool2,self.uframe2)
        for i in range(1,len(primitives2)):
            if i == len(primitives2)-1:
                # this_zone = zone2
                this_zone = -1
            else:
                this_zone = zone2
            option=''
            # speed
            this_speed=int(all_speed[i])

            if primitives2[i]=='movel_fit':
                # robot2
                robt2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveL(robt2,this_speed,'mmsec',this_zone,option)
                # tp_lead.moveL(robt2,this_speed,'msec',this_zone,option)
            elif primitives2[i]=='movec_fit':
                # robot2
                robt_mid2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                robt2 = joint2robtarget(q_bp2[i][1],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveC(robt_mid2,robt2,this_speed,'mmsec',this_zone,option)
            else: #moveJ
                robt2 = joint2robtarget(q_bp2[i][0],robot2,self.group2,self.uframe2,self.utool2)
                tp_lead.moveJ(robt2,this_speed,'%',this_zone)
                # tp_lead.moveJ(robt2,this_speed,'msec',this_zone)
        
        return self.client.execute_motion_program_connect(tp_follow,tp_lead)
    
    def logged_data_analysis(self,robot,df):

        q1=df['J1'].tolist()[1:]
        q2=df['J2'].tolist()[1:]
        q3=df['J3'].tolist()[1:]
        q4=df['J4'].tolist()[1:]
        q5=df['J5'].tolist()[1:]
        q6=df['J6'].tolist()[1:]
        curve_exe_js=np.radians(np.vstack((q1,q2,q3,q4,q5,q6)).T.astype(float))
        timestamp=np.array(df['timestamp'].tolist()[1:]).astype(float)*1e-3 # from msec to sec

        act_speed=[0]
        lam_exec=[0]
        curve_exe=[]
        curve_exe_R=[]
        curve_exe_js_act=[]
        timestamp_act = []
        dont_show_id=[]
        last_cont = False
        for i in range(len(curve_exe_js)):
            this_q = curve_exe_js[i]
            if i>5 and i<len(curve_exe_js)-5:
                # if the recording is not fast enough
                # then having to same logged joint angle
                # do interpolation for estimation
                if np.all(this_q==curve_exe_js[i+1]):
                    dont_show_id=np.append(dont_show_id,i).astype(int)
                    last_cont = True
                    continue

            robot_pose=robot.fwd(this_q)
            curve_exe.append(robot_pose.p)
            curve_exe_R.append(robot_pose.R)
            curve_exe_js_act.append(this_q)
            timestamp_act.append(timestamp[i])
            if i>0:
                lam_exec.append(lam_exec[-1]+np.linalg.norm(curve_exe[-1]-curve_exe[-2]))
            try:
                if timestamp[-1]!=timestamp[-2]:
                    if last_cont:
                        timestep=timestamp[i]-timestamp[i-2]
                    else:
                        timestep=timestamp[i]-timestamp[i-1]
                    act_speed.append(np.linalg.norm(curve_exe[-1]-curve_exe[-2])/timestep)
            except IndexError:
                pass
            last_cont = False

        curve_exe=np.array(curve_exe)
        curve_exe_R=np.array(curve_exe_R)
        curve_exe_js_act=np.array(curve_exe_js_act)
        act_speed = np.array(act_speed)

        return lam_exec, curve_exe, curve_exe_R,curve_exe_js_act, act_speed, timestamp_act
    
    def logged_data_analysis_multimove(self,df,base2_R,base2_p,realrobot=False):
        q1_1=df['J11'].tolist()[1:-1]
        q1_2=df['J12'].tolist()[1:-1]
        q1_3=df['J13'].tolist()[1:-1]
        q1_4=df['J14'].tolist()[1:-1]
        q1_5=df['J15'].tolist()[1:-1]
        q1_6=df['J16'].tolist()[1:-1]
        q2_1=df['J21'].tolist()[1:-1]
        q2_2=df['J22'].tolist()[1:-1]
        q2_3=df['J23'].tolist()[1:-1]
        q2_4=df['J24'].tolist()[1:-1]
        q2_5=df['J25'].tolist()[1:-1]
        q2_6=df['J26'].tolist()[1:-1]
        timestamp=np.round(np.array(df['timestamp'].tolist()[1:]).astype(float)*1e-3,3) # from msec to sec

        curve_exe_js1=np.radians(np.vstack((q1_1,q1_2,q1_3,q1_4,q1_5,q1_6)).T.astype(float))
        curve_exe_js2=np.radians(np.vstack((q2_1,q2_2,q2_3,q2_4,q2_5,q2_6)).T.astype(float))

        if realrobot:
            timestamp, curve_exe_js_all=lfilter(timestamp, np.hstack((curve_exe_js1,curve_exe_js2)))
            curve_exe_js1=curve_exe_js_all[:,:6]
            curve_exe_js2=curve_exe_js_all[:,6:]

        act_speed=[0]
        lam=[0]
        relative_path_exe=[]
        relative_path_exe_R=[]
        curve_exe1=[]
        curve_exe2=[]
        curve_exe_R1=[]
        curve_exe_R2=[]
        curve_exe_js1_act=[]
        curve_exe_js2_act=[]
        timestamp_act = []
        last_cont = False
        for i in range(len(curve_exe_js1)):
            if i>2 and i<len(curve_exe_js1)-2:
                # if the recording is not fast enough
                # then having to same logged joint angle
                # do interpolation for estimation
                if np.all(curve_exe_js1[i]==curve_exe_js1[i+1]) and np.all(curve_exe_js2[i]==curve_exe_js2[i+1]):
                    last_cont = True
                    continue

            curve_exe_js1_act.append(curve_exe_js1[i])
            curve_exe_js2_act.append(curve_exe_js2[i])
            timestamp_act.append(timestamp[i])
            pose1_now=self.robot1.fwd(curve_exe_js1[i])
            pose2_now=self.robot2.fwd(curve_exe_js2[i])
            # curve in robot's own frame
            curve_exe1.append(pose1_now.p)
            curve_exe2.append(pose2_now.p)
            curve_exe_R1.append(pose1_now.R)
            curve_exe_R2.append(pose2_now.R)

            pose2_world_now=self.robot2.fwd(curve_exe_js2[i],world=True)
            relative_path_exe.append(np.dot(pose2_world_now.R.T,pose1_now.p-pose2_world_now.p))
            relative_path_exe_R.append(pose2_world_now.R.T@pose1_now.R)
            
            if i>0:
                lam.append(lam[-1]+np.linalg.norm(relative_path_exe[-1]-relative_path_exe[-2]))
            try:
                if timestamp[-1]!=timestamp[-2]:
                    if last_cont:
                        timestep=timestamp[i]-timestamp[i-2]
                    else:
                        timestep=timestamp[i]-timestamp[i-1]
                    act_speed.append(np.linalg.norm(relative_path_exe[-1]-relative_path_exe[-2])/timestep)
                    
            except IndexError:
                pass
            last_cont = False

        return np.array(lam), np.array(curve_exe1),np.array(curve_exe2), np.array(curve_exe_R1),np.array(curve_exe_R2),np.array(curve_exe_js1_act),\
            np.array(curve_exe_js2_act), np.array(act_speed), np.array(timestamp_act), np.array(relative_path_exe), np.array(relative_path_exe_R)

    def logged_data_analysis_multimove_connect(self,df1,df2,base2_R,base2_p,realrobot=False):
        
        lam_exec1, curve_exe1, curve_exe_R1,curve_exe_js_act1, act_speed1, timestamp_act1 = self.logged_data_analysis(self.robot1,df1)
        lam_exec2, curve_exe2, curve_exe_R2,curve_exe_js_act2, act_speed2, timestamp_act2 = self.logged_data_analysis(self.robot2,df2)

        timestamp_act1=list(timestamp_act1)
        timestamp_act2=list(timestamp_act2)
        curve_exe_js_act1=list(curve_exe_js_act1)
        curve_exe_js_act2=list(curve_exe_js_act2)
        curve_exe_R1=list(curve_exe_R1)
        curve_exe_R2=list(curve_exe_R2)
        curve_exe1=list(curve_exe1)
        curve_exe2=list(curve_exe2)
        dt=0.008
        while timestamp_act1[-1]>timestamp_act2[-1]:
            timestamp_act2.append(timestamp_act2[-1]+0.008)
            curve_exe2.append(curve_exe2[-1])
            curve_exe_R2.append(curve_exe_R2[-1])
            curve_exe_js_act2.append(curve_exe_js_act2[-1])
        while timestamp_act1[-1]<timestamp_act2[-1]:
            timestamp_act1.append(timestamp_act1[-1]+0.008)
            curve_exe1.append(curve_exe1[-1])
            curve_exe_R1.append(curve_exe_R1[-1])
            curve_exe_js_act1.append(curve_exe_js_act1[-1])

        timestamp_comb=[]
        curve_exe1_comb=[]
        curve_exe2_comb=[]
        curve_exe_R1_comb=[]
        curve_exe_R2_comb=[]
        curve_exe_js1_comb=[]
        curve_exe_js2_comb=[]
        speed=[0]
        relative_path_exe=[]
        relative_path_exe_R=[]
        lam=[0]

        for i in range(len(timestamp_act1)):
            if timestamp_act1[i] not in timestamp_act2:
                continue
            id_2 = np.argwhere(timestamp_act2==timestamp_act1[i])[0][0]
            timestamp_comb.append(timestamp_act1[i])
            curve_exe1_comb.append(curve_exe1[i])
            curve_exe2_comb.append(curve_exe2[id_2])
            curve_exe_R1_comb.append(curve_exe_R1[i])
            curve_exe_R2_comb.append(curve_exe_R2[id_2])
            curve_exe_js1_comb.append(curve_exe_js_act1[i])
            curve_exe_js2_comb.append(curve_exe_js_act2[id_2])

            pose1_now=self.robot1.fwd(curve_exe_js_act1[i])
            pose2_world_now=self.robot2.fwd(curve_exe_js_act2[id_2],base2_R,base2_p)
            relative_path_exe.append(np.dot(pose2_world_now.R.T,pose1_now.p-pose2_world_now.p))
            relative_path_exe_R.append(pose2_world_now.R.T@pose1_now.R)

            if i>0:
                lam.append(lam[-1]+np.linalg.norm(relative_path_exe[-1]-relative_path_exe[-2]))
                timestep=timestamp_comb[-1]-timestamp_comb[-2]
                speed.append(np.linalg.norm(relative_path_exe[-1]-relative_path_exe[-2])/timestep)
        
        return np.array(lam),np.array(curve_exe1_comb),np.array(curve_exe2_comb),np.array(curve_exe_R1_comb),np.array(curve_exe_R2_comb),\
            np.array(curve_exe_js1_comb),np.array(curve_exe_js2_comb), np.array(speed), np.array(timestamp_comb), np.array(relative_path_exe), np.array(relative_path_exe_R)

    def chop_extension(self,curve_exe, curve_exe_R,curve_exe_js, speed, timestamp,curve,curve_normal):
        start_idx=np.argmin(np.linalg.norm(curve[0,:]-curve_exe,axis=1))+1
        end_idx=np.argmin(np.linalg.norm(curve[-1,:]-curve_exe,axis=1))

        #make sure extension doesn't introduce error
        if np.linalg.norm(curve_exe[start_idx]-curve[0,:])>0.05:
            start_idx+=1
        if np.linalg.norm(curve_exe[end_idx]-curve[-1,:])>0.05:
            end_idx-=1

        curve_exe=curve_exe[start_idx:end_idx+1]
        curve_exe_js=curve_exe_js[start_idx:end_idx+1]
        curve_exe_R=curve_exe_R[start_idx:end_idx+1]
        speed=speed[start_idx:end_idx+1]
        timestamp=timestamp[start_idx:end_idx+1]
        lam=calc_lam_cs(curve_exe)
        return lam, curve_exe, curve_exe_R,curve_exe_js, speed, timestamp
    
    def chop_extension_dual(self,lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R,p_start,p_end):
        start_idx=np.argmin(np.linalg.norm(p_start-relative_path_exe,axis=1))+1
        end_idx=np.argmin(np.linalg.norm(p_end-relative_path_exe,axis=1))

        #make sure extension doesn't introduce error
        if np.linalg.norm(relative_path_exe[start_idx]-p_start)>0.5:
            start_idx+=1
        if np.linalg.norm(relative_path_exe[end_idx]-p_end)>0.5:
            end_idx-=1

        curve_exe1=curve_exe1[start_idx:end_idx+1]
        curve_exe2=curve_exe2[start_idx:end_idx+1]
        curve_exe_R1=curve_exe_R1[start_idx:end_idx+1]
        curve_exe_R2=curve_exe_R2[start_idx:end_idx+1]
        curve_exe_js1=curve_exe_js1[start_idx:end_idx+1]
        curve_exe_js2=curve_exe_js2[start_idx:end_idx+1]
        timestamp=timestamp[start_idx:end_idx+1]

        relative_path_exe=relative_path_exe[start_idx:end_idx+1]
        relative_path_exe_R=relative_path_exe_R[start_idx:end_idx+1]

        speed=speed[start_idx:end_idx+1]
        lam=calc_lam_cs(relative_path_exe)

        return lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R
    
    def chop_extension_dual_singel(self,lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R,p_start,p_end,curve_chop_target):
        start_idx=np.argmin(np.linalg.norm(p_start-curve_chop_target,axis=1))+1
        end_idx=np.argmin(np.linalg.norm(p_end-curve_chop_target,axis=1))

        #make sure extension doesn't introduce error
        if np.linalg.norm(curve_chop_target[start_idx]-p_start)>0.5:
            start_idx+=1
        if np.linalg.norm(curve_chop_target[end_idx]-p_end)>0.5:
            end_idx-=1

        curve_exe1=curve_exe1[start_idx:end_idx+1]
        curve_exe2=curve_exe2[start_idx:end_idx+1]
        curve_exe_R1=curve_exe_R1[start_idx:end_idx+1]
        curve_exe_R2=curve_exe_R2[start_idx:end_idx+1]
        curve_exe_js1=curve_exe_js1[start_idx:end_idx+1]
        curve_exe_js2=curve_exe_js2[start_idx:end_idx+1]
        timestamp=timestamp[start_idx:end_idx+1]

        relative_path_exe=relative_path_exe[start_idx:end_idx+1]
        relative_path_exe_R=relative_path_exe_R[start_idx:end_idx+1]

        speed=speed[start_idx:end_idx+1]
        lam=calc_lam_cs(relative_path_exe)

        return lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R
    
    def chop_extension_dual_extend(self,lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R,p_start_all,p_end_all,curve_chop_target_all):
        
        start_idx_all=[]
        end_idx_all=[]
        for i in range(len(p_start_all)):
            p_start=p_start_all[i]
            p_end=p_end_all[i]
            curve_chop_target=curve_chop_target_all[i]
            # print(p_start)
            # print(p_end)
            # print(curve_chop_target)

            start_idx=np.argmin(np.linalg.norm(p_start-curve_chop_target,axis=1))+1
            end_idx=np.argmin(np.linalg.norm(p_end-curve_chop_target,axis=1))

            #make sure extension doesn't introduce error
            if np.linalg.norm(curve_chop_target[start_idx]-p_start)>0.5:
                start_idx+=1
            if np.linalg.norm(curve_chop_target[end_idx]-p_end)>0.5:
                end_idx-=1
            
            start_idx_all.append(start_idx)
            end_idx_all.append(end_idx)

        print(start_idx_all)
        print(end_idx_all)
        start_idx=np.min(start_idx_all)
        end_idx=np.max(end_idx_all)

        curve_exe1=curve_exe1[start_idx:end_idx+1]
        curve_exe2=curve_exe2[start_idx:end_idx+1]
        curve_exe_R1=curve_exe_R1[start_idx:end_idx+1]
        curve_exe_R2=curve_exe_R2[start_idx:end_idx+1]
        curve_exe_js1=curve_exe_js1[start_idx:end_idx+1]
        curve_exe_js2=curve_exe_js2[start_idx:end_idx+1]
        timestamp=timestamp[start_idx:end_idx+1]

        relative_path_exe=relative_path_exe[start_idx:end_idx+1]
        relative_path_exe_R=relative_path_exe_R[start_idx:end_idx+1]

        speed=speed[start_idx:end_idx+1]
        lam=calc_lam_cs(relative_path_exe)

        return lam, curve_exe1,curve_exe2,curve_exe_R1,curve_exe_R2,curve_exe_js1,curve_exe_js2, speed, timestamp, relative_path_exe,relative_path_exe_R
    
    def form_relative_path(self,curve_js1,curve_js2,base2_R,base2_p):
        relative_path_exe=[]
        relative_path_exe_R=[]
        curve_exe1=[]
        curve_exe2=[]
        curve_exe_R1=[]
        curve_exe_R2=[]
        for i in range(len(curve_js1)):
            pose1_now=self.robot1.fwd(curve_js1[i])
            pose2_now=self.robot2.fwd(curve_js2[i])

            curve_exe1.append(pose1_now.p)
            curve_exe2.append(pose2_now.p)
            curve_exe_R1.append(pose1_now.R)
            curve_exe_R2.append(pose2_now.R)

            pose2_world_now=self.robot2.fwd(curve_js2[i],world=True)


            relative_path_exe.append(np.dot(pose2_world_now.R.T,pose1_now.p-pose2_world_now.p))
            relative_path_exe_R.append(pose2_world_now.R.T@pose1_now.R)
        return np.array(relative_path_exe),np.array(relative_path_exe_R)

    def calc_robot2_q_from_blade_pose(self,blade_pose,base2_R,base2_p):
        R2=base2_R.T@blade_pose[:3,:3]
        p2=-base2_R.T@(base2_p-blade_pose[:3,-1])

        return self.robot2.inv(p2,R2)[1]
    
    def write_data_to_cmd(self,filename,breakpoints,primitives, p_bp,q_bp):
        p_bp_new=[]
        q_bp_new=[]
        for i in range(len(breakpoints)):
            if len(p_bp[i])==2:
                p_bp_new.append([np.array(p_bp[i][0]),np.array(p_bp[i][1])])
                q_bp_new.append([np.array(q_bp[i][0]),np.array(q_bp[i][1])])
            else:
                p_bp_new.append([np.array(p_bp[i][0])])
                q_bp_new.append([np.array(q_bp[i][0])])
        df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'points':p_bp_new,'q_bp':q_bp_new})
        df.to_csv(filename,header=True,index=False)

    def extend_start_end(self,robot,q_bp,primitives,breakpoints,points_list,extension_start=100,extension_end=100,workpiece=False,robot2=None,q_bp2=None,q_bp2_origin=None,moveL=False):
        
        # dist_bp = np.linalg.norm(np.array(points_list[0][0])-np.array(points_list[1][0]))
        # if dist_bp > extension_start:
        #     dist_bp=extension_start
        # step_to_extend=round(extension_start/dist_bp)
        # extend_step_d_start=float(extension_start)/step_to_extend
        
        ###initial point extension
        if not workpiece:
            pose_start=robot.fwd(q_bp[0][-1])
        else:
            pose_start_world_r2=robot2.fwd(q_bp2_origin[0][-1],world=True)
            pose_start=pose_start_world_r2.inv()*robot.fwd(q_bp[0][-1])
        p_start=pose_start.p
        R_start=pose_start.R
        if not workpiece:
            pose_end=robot.fwd(q_bp[1][-1])
        else:
            pose_end_world_r2=robot2.fwd(q_bp2_origin[1][-1],world=True)
            pose_end=pose_end_world_r2.inv()*robot.fwd(q_bp[1][-1])
        p_end=pose_end.p
        R_end=pose_end.R
        if primitives[1]=='movel_fit' or moveL:
            #find new start point
            slope_p=p_end-p_start
            slope_p=slope_p/np.linalg.norm(slope_p)
            p_start_new=p_start-extension_start*slope_p        ###extend 5cm backward

            #find new start orientation
            k,theta=R2rot(R_end@R_start.T)
            theta_new=-extension_start*theta/np.linalg.norm(p_end-p_start)

            R_start_new=rot(k,theta_new)@R_start
            
            # solve invkin for initial point
            if not workpiece:
                try:
                    q_bp.insert(0,[car2js(robot,q_bp[0][0],p_start_new,R_start_new)[0]])
                    points_list.insert(0,[p_start_new])
                    primitives.insert(1,'movel_fit')
                except:
                    q_bp.insert(0,[car2js(robot,q_bp[0][0],p_start_new,R_start)[0]])
                    points_list.insert(0,[p_start_new])
                    primitives.insert(1,'movel_fit')
            else:
                T_start_new_world_r2=robot2.fwd(q_bp2[0][-1],world=True)
                p_start_new_world_r1 = T_start_new_world_r2.R@p_start_new+T_start_new_world_r2.p
                R_start_new_world_r1 = T_start_new_world_r2.R@R_start_new
                q_bp.insert(0,[car2js(robot,q_bp[0][0],p_start_new_world_r1,R_start_new_world_r1)[0]])
                points_list.insert(0,[p_start_new_world_r1])
                primitives.insert(1,'movel_fit')

        elif  primitives[1]=='movec_fit':
            
            if not workpiece:
                pose_mid=robot.fwd(q_bp[1][0])
            else:
                pose_mid_world_r2=robot2.fwd(q_bp2_origin[1][0],world=True)
                pose_mid=pose_mid_world_r2.inv()*robot.fwd(q_bp[1][0])

            #define circle first
            p_mid=pose_mid.p
            R_mid=pose_mid.R

            center, radius=circle_from_3point(p_start,p_end,p_mid)

            #find desired rotation angle
            angle=extension_start/radius

            #find new start point
            plane_N=np.cross(p_end-center,p_start-center)
            plane_N=plane_N/np.linalg.norm(plane_N)
            R_temp=rot(plane_N,angle)
            p_start_new=center+R_temp@(p_start-center)

            #modify mid point to be in the middle of new start and old end (to avoid RS circle uncertain error)
            modified_bp=arc_from_3point(p_start_new,p_end,p_mid,N=3)
            
            #find new start orientation
            # k,theta=R2rot(R_end@R_start.T)
            k,theta=R2rot(R_end.T@R_start)
            theta_new=-extension_start*theta/np.linalg.norm(p_end-p_start)
            # R_start_new=rot(k,theta_new)@R_start
            # R_mid_new=rot(k,theta_new/2)@R_start
            R_start_new=R_start@rot(k,theta_new)
            R_mid_new=R_start@rot(k,theta_new/2)

            #solve invkin for initial point
            if not workpiece:
                points_list[1][0]=modified_bp[1]
                points_list[0][0]=p_start_new
                q_bp[0][0]=car2js(robot,q_bp[0][0],p_start_new,R_start_new)[0]
                q_bp[1][0]=car2js(robot,q_bp[1][0],points_list[1][0],R_start)[0] ## kind of a compromise here
            else:
                T_start_new_world_r2=robot2.fwd(q_bp2[0][0],world=True)
                T_mid_new_world_r2=robot2.fwd(q_bp2[1][0],world=True)
                p_start_new_world_r1 = T_start_new_world_r2.R@p_start_new+T_start_new_world_r2.p
                p_mid_new_world_r1 = T_mid_new_world_r2.R@modified_bp[1]+T_mid_new_world_r2.p
                R_start_new_world_r1 = T_start_new_world_r2.R@R_start_new
                R_mid_world_r1 = T_mid_new_world_r2.R@R_start ## kind of a compromise here

                points_list[0][0]=p_start_new_world_r1
                points_list[1][0]=p_mid_new_world_r1
                q_bp[0][0]=car2js(robot,q_bp[0][0],points_list[0][0],R_start_new_world_r1)[0]
                q_bp[1][0]=car2js(robot,q_bp[1][0],points_list[1][0],R_mid_world_r1)[0]

        else:
            #find new start point
            J_start=robot.jacobian(q_bp[0][0])
            qdot=q_bp[1][0]-q_bp[0][0]
            v=J_start[3:,:]@qdot
            t=extension_start/np.linalg.norm(v)
            
            q_bp[0][0]=q_bp[0][0]+qdot*t
            points_list[0][0]=robot.fwd(q_bp[0][0]).p

        ###end point extension
        if not workpiece:
            pose_start=robot.fwd(q_bp[-2][-1])
        else:
            pose_start_world_r2=robot2.fwd(q_bp2_origin[-2][-1],world=True)
            pose_start=pose_start_world_r2.inv()*robot.fwd(q_bp[-2][-1])
        p_start=pose_start.p
        R_start=pose_start.R
        if not workpiece:
            pose_end=robot.fwd(q_bp[-1][-1])
        else:
            pose_end_world_r2=robot2.fwd(q_bp2_origin[-1][-1],world=True)
            pose_end=pose_end_world_r2.inv()*robot.fwd(q_bp[-1][-1])
        p_end=pose_end.p
        R_end=pose_end.R

        # dist_bp = np.linalg.norm(np.array(points_list[-1][-1])-np.array(points_list[-2][-1]))
        # if dist_bp > extension_end:
        #     dist_bp=extension_end
        # step_to_extend=round(extension_end/dist_bp)
        # extend_step_d_end=float(extension_end)/step_to_extend

        if primitives[-1]=='movel_fit' or moveL:
            #find new end point
            slope_p=p_end-p_start
            slope_p=slope_p/np.linalg.norm(slope_p)
            p_end_new=p_end+extension_end*slope_p        ###extend 5cm backward

            #find new end orientation
            k,theta=R2rot(R_end@R_start.T)
            theta_new=extension_end*theta/np.linalg.norm(p_end-p_start)
            R_end_new=rot(k,theta_new)@R_end

            # solve invkin for initial point
            if not workpiece:
                try:
                    q_bp.append([car2js(robot,q_bp[-1][0],p_end_new,R_end_new)[0]])
                    points_list.append([p_end_new])
                    primitives.append('movel_fit')
                except:
                    q_bp.append([car2js(robot,q_bp[-1][0],p_end_new,R_end)[0]])
                    points_list.append([p_end_new])
                    primitives.append('movel_fit')
            else:
                p_end_new_world_r2=robot2.fwd(q_bp2[-1][-1],world=True)
                p_end_new_world_r1 = p_end_new_world_r2.R@p_end_new+p_end_new_world_r2.p
                R_end_new_world_r1 = p_end_new_world_r2.R@R_end_new
                q_bp.append([car2js(robot,q_bp[-1][0],p_end_new_world_r1,R_end_new_world_r1)[0]])
                points_list.append([p_end_new_world_r1])
                primitives.append('movel_fit')

        elif  primitives[-1]=='movec_fit':
            #define circle first
            if not workpiece:
                pose_mid=robot.fwd(q_bp[-1][0])
            else:
                pose_mid_world_r2=robot2.fwd(q_bp2_origin[-1][0],world=True)
                pose_mid=pose_mid_world_r2.inv()*robot.fwd(q_bp[-1][0])
            p_mid=pose_mid.p
            R_mid=pose_mid.R
            center, radius=circle_from_3point(p_start,p_end,p_mid)
            # ax = plt.axes(projection='3d')
            # ax.scatter3D(p_start[0], p_start[1],p_start[2])
            # ax.scatter3D(p_mid[0], p_mid[1],p_mid[2])
            # ax.scatter3D(p_end[0], p_end[1],p_end[2])
            # ax.scatter3D(center[0], center[1],center[2])
            # plt.show()
            # print(radius)

            #find desired rotation angle
            angle=extension_end/radius
            # print(np.degrees(angle))

            angle=min([angle,np.pi/3*2])

            #find new end point
            plane_N=np.cross(p_start-center,p_end-center)
            plane_N=plane_N/np.linalg.norm(plane_N)
            R_temp=rot(plane_N,angle)
            p_end_new=center+R_temp@(p_end-center)

            #modify mid point to be in the middle of new end and old start (to avoid RS circle uncertain error)
            modified_bp=arc_from_3point(p_start,p_end_new,p_mid,N=3)
            
            #find new end orientation
            k,theta=R2rot(R_end@R_start.T)
            theta_new=extension_end*theta/np.linalg.norm(p_end-p_start)
            R_end_new=rot(k,theta_new)@R_end
            R_mid_new=rot(k,theta_new/2)@R_end

            #solve invkin for initial point
            if not workpiece:
                points_list[-1][0]=modified_bp[1]
                points_list[-1][-1]=p_end_new   #midpoint not changed
                q_bp[-1][0]=car2js(robot,q_bp[-1][0],points_list[-1][0],R_end)[0]
                q_bp[-1][-1]=car2js(robot,q_bp[-1][-1],p_end_new,R_end_new)[0]
            else:
                T_mid_new_world_r2=robot2.fwd(q_bp2[-1][0],world=True)
                T_end_new_world_r2=robot2.fwd(q_bp2[-1][-1],world=True)
                p_mid_new_world_r1 = T_mid_new_world_r2.R@modified_bp[1]+T_mid_new_world_r2.p
                p_end_new_world_r1 = T_end_new_world_r2.R@p_end_new+T_end_new_world_r2.p
                R_mid_new_world_r1 = T_mid_new_world_r2.R@R_end ## kind of a compromise here
                R_end_world_r1 = T_end_new_world_r2.R@R_end_new

                points_list[-1][0]=p_mid_new_world_r1
                points_list[-1][-1]=p_end_new_world_r1
                q_bp[-1][0]=car2js(robot,q_bp[-1][0],points_list[-1][0],R_mid_new_world_r1)[0]
                q_bp[-1][-1]=car2js(robot,q_bp[-1][-1],points_list[-1][-1],R_end_world_r1)[0]

        else:
            #find new end point
            J_end=robot.jacobian(q_bp[-1][0])
            qdot=q_bp[-1][0]-q_bp[-2][0]
            dlam=np.linalg.norm(J_end[3:,:]@qdot)
            t=extension_end/dlam

            q_bp[-1][0]=q_bp[-1][-1]+qdot*t
            points_list[-1][0]=robot.fwd(q_bp[-1][-1]).p

        return primitives,points_list,q_bp
    
    def extend_start_end_qp(self,robot,q_bp,primitives,breakpoints,points_list,extension_start=100,extension_end=100):
        
        dist_bp = np.linalg.norm(np.array(points_list[0][0])-np.array(points_list[1][0]))
        if dist_bp > extension_start:
            dist_bp=extension_start
        step_to_extend=round(extension_start/dist_bp)
        extend_step_d_start=float(extension_start)/step_to_extend
        
        ###initial point extension
        pose_start=robot.fwd(q_bp[0][-1])
        p_start=pose_start.p
        R_start=pose_start.R
        pose_end=robot.fwd(q_bp[1][-1])
        p_end=pose_end.p
        R_end=pose_end.R

        if primitives[1]=='movel_fit':
            #find new start point
            slope_p=p_end-p_start
            slope_p=slope_p/np.linalg.norm(slope_p)
            p_start_new=p_start-extension_start*slope_p        ###extend 5cm backward

            #find new start orientation
            k,theta=R2rot(R_end@R_start.T)
            theta_new=-extension_start*theta/np.linalg.norm(p_end-p_start)
            # R_start_new=rot(k,theta_new)@R_start

            # adding extension with uniform space
            steps_opt=round(extension_start/np.linalg.norm(pose_start.p-pose_end.p))*500
            steps=round(extension_start/np.linalg.norm(pose_start.p-pose_end.p))
            relative_path=[]
            extend_step=extension_start/steps_opt
            for i in range(1,steps_opt+1):
                pn=np.array([])
                p_extend=p_start-i*extend_step*slope_p
                theta_extend=-np.linalg.norm(p_extend-p_start)*theta/np.linalg.norm(p_end-p_start)
                R_extend=rot(k,theta_extend)@R_start
                pn = np.append(pn,p_extend)
                pn = np.append(pn,R_extend[:,-1])
                relative_path.append(pn)
            relative_path=np.array(relative_path)
            ## extend in relative
            opt=lambda_opt(relative_path[:,:3],relative_path[:,3:],robot1=robot,steps=steps_opt)
            q_out=opt.single_arm_stepwise_optimize(q_bp[0][-1])
            # adding extension with uniform space
            for i in range(1,steps+1):
                points_list.insert(0,[robot.fwd(q_out[int(i*(steps_opt/steps))-1]).p])
                q_bp.insert(0,[q_out[int(i*(steps_opt/steps))-1]])
                primitives.insert(1,'movel_fit')

        elif  primitives[1]=='movec_fit':
            #define circle first
            pose_mid=robot.fwd(q_bp[0][0])
            p_mid=pose_mid.p
            R_mid=pose_mid.R
            center, radius=circle_from_3point(p_start,p_end,p_mid)

            #find desired rotation angle
            angle=extension_start/radius

            #find new start point
            plane_N=np.cross(p_end-center,p_start-center)
            plane_N=plane_N/np.linalg.norm(plane_N)
            R_temp=rot(plane_N,angle)
            p_start_new=center+R_temp@(p_start-center)

            #find new start orientation
            k,theta=R2rot(R_end@R_start.T)
            theta_new=-extension_start*theta/np.linalg.norm(p_end-p_start)
            R_start_new=rot(k,theta_new)@R_start

            #solve invkin for initial point
            points_list[0][0]=p_start_new
            q_bp[0][0]=car2js(robot,q_bp[0][0],p_start_new,R_start_new)[0]

        else:
            #find new start point
            J_start=robot.jacobian(q_bp[0][0])
            qdot=q_bp[1][0]-q_bp[0][0]
            v=J_start[3:,:]@qdot
            t=extension_start/np.linalg.norm(v)
            
            q_bp[0][0]=q_bp[0][0]+qdot*t
            points_list[0][0]=robot.fwd(q_bp[0][0]).p

        ###end point extension
        pose_start=robot.fwd(q_bp[-2][-1])
        p_start=pose_start.p
        R_start=pose_start.R
        pose_end=robot.fwd(q_bp[-1][-1])
        p_end=pose_end.p
        R_end=pose_end.R

        dist_bp = np.linalg.norm(np.array(points_list[-1][-1])-np.array(points_list[-2][-1]))
        if dist_bp > extension_end:
            dist_bp=extension_end
        step_to_extend=round(extension_end/dist_bp)
        extend_step_d_end=float(extension_end)/step_to_extend

        if primitives[-1]=='movel_fit':
            #find new end point
            slope_p=p_end-p_start
            slope_p=slope_p/np.linalg.norm(slope_p)
            # p_end_new=p_end+extension_d*slope_p        ###extend 5cm backward

            #find new end orientation
            k,theta=R2rot(R_end@R_start.T)
            # theta_new=extension_d*theta/np.linalg.norm(p_end-p_start)
            # R_end_new=rot(k,theta_new)@R_end

            # adding extension with uniform space
            steps_opt=round(extension_end/np.linalg.norm(pose_start.p-pose_end.p))*500
            steps=round(extension_end/np.linalg.norm(pose_start.p-pose_end.p))
            relative_path=[]
            extend_step=extension_end/steps_opt
            for i in range(1,steps_opt+1):
                pn=np.array([])
                p_extend=p_end+i*extend_step*slope_p
                theta_extend=np.linalg.norm(p_extend-p_end)*theta/np.linalg.norm(p_end-p_start)
                R_extend=rot(k,theta_extend)@R_end
                pn = np.append(pn,p_extend)
                pn = np.append(pn,R_extend[:,-1])
                relative_path.append(pn)
            relative_path=np.array(relative_path)
            ## extend in relative
            opt=lambda_opt(relative_path[:,:3],relative_path[:,3:],robot1=robot,steps=steps_opt)
            q_out=opt.single_arm_stepwise_optimize(q_bp[-1][-1])
            # adding extension with uniform space
            for i in range(1,steps+1):
                points_list.append([robot.fwd(q_out[int(i*(steps_opt/steps))-1]).p])
                q_bp.append([q_out[int(i*(steps_opt/steps))-1]])
                primitives.append('movel_fit')

        elif  primitives[1]=='movec_fit':
            #define circle first
            pose_mid=robot.fwd(q_bp[-1][0])
            p_mid=pose_mid.p
            R_mid=pose_mid.R
            center, radius=circle_from_3point(p_start,p_end,p_mid)

            #find desired rotation angle
            angle=extension_end/radius

            #find new end point
            plane_N=np.cross(p_start-center,p_end-center)
            plane_N=plane_N/np.linalg.norm(plane_N)
            R_temp=rot(plane_N,angle)
            p_end_new=center+R_temp@(p_end-center)

            #find new end orientation
            k,theta=R2rot(R_end@R_start.T)
            theta_new=extension_end*theta/np.linalg.norm(p_end-p_start)
            R_end_new=rot(k,theta_new)@R_end

            #solve invkin for end point
            q_bp[-1][-1]=car2js(robot,q_bp[-1][-1],p_end_new,R_end_new)[0]
            points_list[-1][-1]=p_end_new   #midpoint not changed

        else:
            #find new end point
            J_end=robot.jacobian(q_bp[-1][0])
            qdot=q_bp[-1][0]-q_bp[-2][0]
            v=J_end[3:,:]@qdot
            t=extension_end/np.linalg.norm(v)
            
            q_bp[-1][-1]=q_bp[-1][-1]+qdot*t
            points_list[-1][0]=robot.fwd(q_bp[-1][-1]).p

        return primitives,points_list,q_bp

    def extend_start_end_relative(self,robot1,q_bp1,primitives1,p_bp1,robot2,q_bp2,primitives2,p_bp2,base2_T,extension_d,steps_opt,steps):
        ##### extend tool start
        pose_start=(base2_T*robot2.fwd(q_bp2[0][-1])).inv()*robot1.fwd(q_bp1[0][-1])
        p_start=pose_start.p
        R_start=pose_start.R
        pose_end=(base2_T*robot2.fwd(q_bp2[1][-1])).inv()*robot1.fwd(q_bp1[1][-1])
        p_end=pose_end.p
        R_end=pose_end.R
        #find new start point
        slope_p=p_end-p_start
        slope_p=slope_p/np.linalg.norm(slope_p)
        #find new start orientation
        k,theta=R2rot(R_end@R_start.T)
        # adding extension with uniform space
        relative_path=[]
        extend_step=extension_d/steps_opt
        for i in range(1,steps_opt+1):
            pn=np.array([])
            p_extend=p_start-i*extend_step*slope_p
            theta_extend=-np.linalg.norm(p_extend-p_start)*theta/np.linalg.norm(p_end-p_start)
            R_extend=rot(k,theta_extend)@R_start
            pn = np.append(pn,p_extend)
            pn = np.append(pn,R_extend[:,-1])
            relative_path.append(pn)
        relative_path=np.array(relative_path)
        ## extend in relative
        opt=lambda_opt(relative_path[:,:3],relative_path[:,3:],robot1=robot1,robot2=robot2,base2_R=base2_T.R,base2_p=base2_T.p,steps=steps_opt)
        q_out1, q_out2=opt.dual_arm_stepwise_optimize(q_bp1[0][-1],q_bp2[0][-1])
        # adding extension with uniform space
        for i in range(1,steps+1):
            p_bp1.insert(0,[robot1.fwd(q_out1[int(i*(steps_opt/steps))-1]).p])
            p_bp2.insert(0,[robot2.fwd(q_out2[int(i*(steps_opt/steps))-1]).p])
            q_bp1.insert(0,[q_out1[int(i*(steps_opt/steps))-1]])
            q_bp2.insert(0,[q_out2[int(i*(steps_opt/steps))-1]])
            primitives1.insert(1,'movel_fit')
            primitives2.insert(1,'movel_fit')
        ##########################################
        ##### extend tool start
        pose_start=(base2_T*robot2.fwd(q_bp2[-2][-1])).inv()*robot1.fwd(q_bp1[-2][-1])
        p_start=pose_start.p
        R_start=pose_start.R
        pose_end=(base2_T*robot2.fwd(q_bp2[-1][-1])).inv()*robot1.fwd(q_bp1[-1][-1])
        p_end=pose_end.p
        R_end=pose_end.R
        #find new start point
        slope_p=p_end-p_start
        slope_p=slope_p/np.linalg.norm(slope_p)
        #find new start orientation
        k,theta=R2rot(R_end@R_start.T)
        # adding extension with uniform space
        relative_path=[]
        extend_step=extension_d/steps_opt
        for i in range(1,steps_opt+1):
            pn=np.array([])
            p_extend=p_end+i*extend_step*slope_p
            theta_extend=np.linalg.norm(p_extend-p_end)*theta/np.linalg.norm(p_end-p_start)
            R_extend=rot(k,theta_extend)@R_end
            pn = np.append(pn,p_extend)
            pn = np.append(pn,R_extend[:,-1])
            relative_path.append(pn)
        relative_path=np.array(relative_path)
        ## extend in relative
        opt=lambda_opt(relative_path[:,:3],relative_path[:,3:],robot1=robot1,robot2=robot2,base2_R=base2_T.R,base2_p=base2_T.p,steps=steps_opt)
        q_out1, q_out2=opt.dual_arm_stepwise_optimize(q_bp1[-1][-1],q_bp2[-1][-1])
        # adding extension with uniform space
        for i in range(1,steps+1):
            p_bp1.append([robot1.fwd(q_out1[int(i*(steps_opt/steps))-1]).p])
            p_bp2.append([robot2.fwd(q_out2[int(i*(steps_opt/steps))-1]).p])
            q_bp1.append([q_out1[int(i*(steps_opt/steps))-1]])
            q_bp2.append([q_out2[int(i*(steps_opt/steps))-1]])
            primitives1.append('movel_fit')
            primitives2.append('movel_fit')
        
        return primitives1,p_bp1,q_bp1,primitives2,p_bp2,q_bp2
    
    def extend_dual_relative(self,robot1,p_bp1,q_bp1,primitives1,robot2,p_bp2,q_bp2,primitives2,breakpoints,base2_T,extension_d=100):
        #extend porpotionally
        d1_start=np.linalg.norm(p_bp1[1][-1]-p_bp1[0][-1])
        d2_start=np.linalg.norm(p_bp2[1][-1]-p_bp2[0][-1])
        d1_end=np.linalg.norm(p_bp1[-1][-1]-p_bp1[-2][-1])
        d2_end=np.linalg.norm(p_bp2[-1][-1]-p_bp2[-2][-1])

        if (d2_start > 1e-9) and (d2_end > 1e-9):
            pose_start=(base2_T*robot2.fwd(q_bp2[0][-1])).inv()*robot1.fwd(q_bp1[0][-1])
            pose_end=(base2_T*robot2.fwd(q_bp2[1][-1])).inv()*robot1.fwd(q_bp1[1][-1])
            steps_opt=round(extension_d/np.linalg.norm(pose_start.p-pose_end.p))*500
            steps=round(extension_d/np.linalg.norm(pose_start.p-pose_end.p))
            primitives1,p_bp1,q_bp1,primitives2,p_bp2,q_bp2=\
                self.extend_start_end_relative(robot1,q_bp1,primitives1,p_bp1,robot2,q_bp2,primitives2,p_bp2,base2_T,extension_d,steps_opt,steps)
            step_to_extend_end=steps
            ### How much steps it need to extend start and end
            # step_to_extend_start=round(extension_d/d2_start) if d2_start < extension_d else 1
            # step_to_extend_end=round(extension_d/d2_end) if d2_end < extension_d else 1
            # ### First, extend the leader (workpiece robot).
            # primitives2,p_bp2,q_bp2=self.extend_start_end(robot2,q_bp2,primitives2,breakpoints,p_bp2,extension_start=extension_d,extension_end=extension_d)
            # ### Then, extend the follower (tool robot) in the workpiece (i.e. leader robot workpiece) frame.
            # # primitives1,p_bp1,q_bp1=self.extend_start_end_relative(robot1,q_bp1,primitives1,p_bp1,robot2,q_bp2,p_bp2,base2_T,step_to_extend_start,step_to_extend_end)
            # primitives1,p_bp1,q_bp1=self.extend_start_end(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_d*d1_start/d2_start,extension_end=extension_d*d1_end/d2_end)
            
        else: # if the leader robot barely move
            ### How much steps it need to extend start and end
            step_to_extend_start=round(extension_d/d1_start) if d1_start < extension_d else 1
            step_to_extend_end=round(extension_d/d1_end) if d1_end < extension_d else 1
            primitives1,p_bp1,q_bp1=self.extend_start_end(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_d,extension_end=extension_d)
            add_num = len(p_bp1)-len(p_bp2)
            for i in range(add_num):
                p_bp2.append(p_bp2[-1])
                q_bp2.append(q_bp2[-1])
                primitives2.append(primitives2[-1])

        return p_bp1,q_bp1,p_bp2,q_bp2,step_to_extend_end

    def extend_dual(self,robot1,p_bp1,q_bp1,primitives1,robot2,p_bp2,q_bp2,primitives2,breakpoints,base2_T,extension_start=100,extension_end=100):
        
        #extend porpotionally
        d1_start=np.linalg.norm(p_bp1[1][-1]-p_bp1[0][-1])
        d2_start=np.linalg.norm(p_bp2[1][-1]-p_bp2[0][-1])
        d1_end=np.linalg.norm(p_bp1[-1][-1]-p_bp1[-2][-1])
        d2_end=np.linalg.norm(p_bp2[-1][-1]-p_bp2[-2][-1])

        
        ### First, extend the leader (workpiece robot).
        q_bp2_origin=deepcopy(q_bp2)
        # primitives2,p_bp2,q_bp2=self.extend_start_end(robot2,q_bp2,primitives2,breakpoints,p_bp2,extension_start=extension_start,extension_end=extension_end)
        primitives2,p_bp2,q_bp2=self.extend_start_end(robot2,q_bp2,primitives2,breakpoints,p_bp2,extension_start=extension_start,extension_end=extension_end,moveL=True)
        ### Then, extend the follower (tool robot) in the workpiece (i.e. leader robot workpiece) frame.
        # primitives1,p_bp1,q_bp1=self.extend_start_end(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_start*d1_start/d2_start,extension_end=extension_end*d1_end/d2_end,workpiece=True,robot2=robot2,q_bp2=q_bp2,q_bp2_origin=q_bp2_origin)
        # primitives1,p_bp1,q_bp1=self.extend_start_end(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_start,extension_end=extension_end,workpiece=True,robot2=robot2,q_bp2=q_bp2,q_bp2_origin=q_bp2_origin,moveL=True)
        # primitives1,p_bp1,q_bp1=self.extend_start_end(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_start,extension_end=extension_end,workpiece=True,robot2=robot2,q_bp2=q_bp2,q_bp2_origin=q_bp2_origin,moveL=True)
        primitives1,p_bp1,q_bp1=self.extend_start_end(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_start*d1_start/d2_start,extension_end=extension_end*d1_end/d2_end,moveL=True)
       
        
        # primitives1,p_bp1,q_bp1=self.extend_start_end(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_d,extension_end=extension_d)

        # ### First, extend the leader (workpiece robot).
        # primitives2,p_bp2,q_bp2=self.extend_start_end_qp(robot2,q_bp2,primitives2,breakpoints,p_bp2,extension_start=extension_d,extension_end=extension_d)
        # ### Then, extend the follower (tool robot) in the workpiece (i.e. leader robot workpiece) frame.
        # primitives1,p_bp1,q_bp1=self.extend_start_end_qp(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_d*d1_start/d2_start,extension_end=extension_d*d1_end/d2_end)
        # # primitives1,p_bp1,q_bp1=self.extend_start_end(robot1,q_bp1,primitives1,breakpoints,p_bp1,extension_start=extension_d,extension_end=extension_d)


        return p_bp1,q_bp1,p_bp2,q_bp2
    
    def extract_data_from_cmd(self,filename):
        data = read_csv(filename)

        if 'breakpoints' in data.keys():
            breakpoints=np.array(data['breakpoints'].tolist())
        else:
            breakpoints=[]
        primitives=data['primitives'].tolist()
        if 'points' in data.keys():
            points=data['points'].tolist()
        else:
            points=data['p_bp'].tolist()
        qs=data['q_bp'].tolist()
        if 'speed' in data.keys():
            speed=data['speed'].tolist()
        else:
            speed=[]
        p_bp=[]
        q_bp=[]
        for i in range(len(primitives)):
            if primitives[i]=='movel_fit':
                point=self.extract_points(primitives[i],points[i])
                p_bp.append([np.array(point)])
                q=self.extract_points(primitives[i],qs[i])
                q_bp.append([q])


            elif primitives[i]=='movec_fit':
                point1,point2=self.extract_points(primitives[i],points[i])
                p_bp.append([np.array(point1),np.array(point2)])
                q1,q2=self.extract_points(primitives[i],qs[i])
                q_bp.append([q1,q2])

            else:
                point=self.extract_points(primitives[i],points[i])
                p_bp.append([point])
                q=self.extract_points(primitives[i],qs[i])
                q_bp.append([q])

        return breakpoints,primitives, p_bp,q_bp,speed

    def extract_points(self,primitive_type,points):
        if primitive_type=='movec_fit':
            endpoints=points[8:-3].split('array')
            endpoint1=endpoints[0][:-4].split(',')
            endpoint2=endpoints[1][2:].split(',')

            return list(map(float, endpoint1)),list(map(float, endpoint2))
        else:
            if points[1] == '[':
                endpoint=points[2:-2].split(',')
                return np.array(list(map(float, endpoint)))
            else:
                endpoint=points[8:-3].split(',')
                return np.array(list(map(float, endpoint)))
MODULE egm
            
    VAR egmident egmID1;
    VAR egmstate egmSt1;
    
    CONST egm_minmax egm_minmax_lin1:=[-1,1]; !in mm
    CONST egm_minmax egm_minmax_rot1:=[-2,2];! in degees
    VAR pose posecorTable:=[[0,0,0],[1,0,0,0]];
    VAR pose posesenTable:=[[0,0,0],[1,0,0,0]];
    
    PERS tooldata sprayer := [TRUE,[[75.0, 0.0, 493.30126953125],[0.9659258127212524, 0.0, 0.258819043636322, 0.0]],[1.0,[0.0, 0.0, 0.0010000000474974513],[1.0, 0.0, 0.0, 0.0],0.0,0.0,0.0]];
    
    CONST egm_minmax egm_minmax_joint1:=[-0.5,0.5];
                
    PROC main()
        VAR jointtarget joints;
        
        ! "Move" a tiny amount to start EGM
        joints:= CJointT();
        !joints:=[[0,0,23,67,-50,0],[0,0,0,0,0,0]];
        !joints.robax.rax_6 := joints.robax.rax_6 + .0001;
        MoveAbsj joints, v1000, fine, tool0;
    
        StartEGM;
    
        EGMActJoint egmID1 \Tool:=tool0 \WObj:=wobj0, \J1:=egm_minmax_joint1 \J2:=egm_minmax_joint1 \J3:=egm_minmax_joint1
        \J4:=egm_minmax_joint1 \J5:=egm_minmax_joint1 \J6:=egm_minmax_joint1 \LpFilter:=100 \Samplerate:=4 \MaxPosDeviation:=1000 \MaxSpeedDeviation:=1000;            
        EGMRunJoint egmID1, EGM_STOP_HOLD \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=2000000 \RampInTime:=0.01 \PosCorrGain:=1;
        
        !EGMActPose egmID1 \Tool:=sprayer \WObj:=wobj0, posecorTable, EGM_FRAME_BASE, posesenTable, EGM_FRAME_BASE 
        !    \x:=egm_minmax_lin1 \y:=egm_minmax_lin1 \z:=egm_minmax_lin1 \rx:=egm_minmax_rot1
        !    \ry:=egm_minmax_rot1 \rz:=egm_minmax_rot1 \LpFilter:=100
        !    \Samplerate:=4, \MaxPosDeviation:=1000 \MaxSpeedDeviation:=1000;
        !EGMRunPose egmID1, EGM_STOP_HOLD \x \y \z \rx \ry \rz \CondTime:=2000000 \RampInTime:=0.01 \PosCorrGain:=1;

            
        WaitUntil FALSE;
            
        ExitCycle;           
        
    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "EGM UDP Command Timeout, Restarting!";
            WaitTime 1;
            ExitCycle;
        ELSE            
            RAISE;
        ENDIF
    ENDPROC
    
    
    
    PROC StartEGM()
        
        !This call to EGMReset seems to be problematic.
        !It is shown in all documentation. Is it really necessary?
        !EGMReset egmID1;
        
        EGMGetId egmID1;
        egmSt1 := EGMGetState(egmID1);        
        
        IF egmSt1 <= EGM_STATE_CONNECTED THEN            
            EGMSetupUC ROB_1, egmID1, "conf1", "UCdevice:" \Joint \CommTimeout:=10000;
            !EGMSetupUC ROB_1, egmID1, "conf1", "UCdevice:" \Pose \CommTimeout:=10000;
        ENDIF
        
        EGMStreamStart egmID1;
    ENDPROC   
        
ENDMODULE
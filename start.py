from PyQt5.QtCore import QDateTime,Qt,QTimer,QDir,QObject,QRunnable,pyqtSignal,pyqtSlot,QThread
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QDoubleSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget,QFileDialog, QMessageBox)
from PyQt5.QtGui import QPixmap
import sys,os,time,traceback,platform,subprocess
from pathlib import Path
from pandas import *


from toolbox.robots_def import *
from redundancy_resolution.redundancy_resolution import *
from cmd_gen.cmd_gen import *
from toolbox.abb_utils import *
from toolbox.fanuc_utils import *
from motion_update.motion_update import *
from toolbox.tes_env import *

def msgbtn(i):
    print ("Button pressed is:",i.text())

def ping(host):
    """
    Returns True if host (str) responds to a ping request.
    Remember that a host may not respond to a ping (ICMP) request even if the host name is valid.
    """

    # Option for the number of packets as a function of
    param = '-n' if platform.system().lower()=='windows' else '-c'

    # Building the command. Ex: "ping -c 1 google.com"
    command = ['ping', param, '1', host]

    return subprocess.call(command) == 0


# Timer Objecy
class Timer(QObject):
    finished = pyqtSignal(float)
    progress = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.timer_flag=False

    def run(self):
        st=time.time()
        self.timer_flag=True
        while self.timer_flag:
            time.sleep(1)
            this_stamp=time.time()
            self.duration=this_stamp-st
            self.progress.emit(self.duration)
        self.finished.emit(self.duration)
    
    def stop(self):
        print("Timer stop")
        self.timer_flag=False

# Worker Object
class Worker(QObject):
    proc_finished=pyqtSignal()
    finished = pyqtSignal()
    progress = pyqtSignal(float)
    result = pyqtSignal('QVariant')

    def __init__(self, function, *args):
        super().__init__()
        self.function = function
        self.args = args
        self.duration=None

    def run(self):
        try:
            res = list(self.function(*self.args))
        except:
            traceback.print_exc()
            print('function returned None, check error in function call')
            res = []
        self.proc_finished.emit()
        while self.duration is None:
            time.sleep(0.01)
        res.append(self.duration)
        self.result.emit(res)
        self.finished.emit()
    
    def gettime(self,duration):
        self.duration=duration
    
def setup_worker_timer(worker,worker_thread,timer,timer_thread,prog_func,res_func):
    
    # setup timer
    timer.moveToThread(timer_thread)
    timer_thread.started.connect(timer.run)
    timer.progress.connect(prog_func)
    timer.finished.connect(worker.gettime)
    timer.finished.connect(timer_thread.quit)
    timer.finished.connect(timer.deleteLater)
    timer_thread.finished.connect(timer_thread.deleteLater)

    # setup worker and thread
    worker.moveToThread(worker_thread)
    worker_thread.started.connect(worker.run)
    worker.proc_finished.connect(lambda: timer.stop()) # stop the timer when finish
    worker.finished.connect(worker_thread.quit)
    worker.finished.connect(worker.deleteLater)
    worker.result.connect(res_func)
    worker_thread.finished.connect(worker_thread.deleteLater)

    return worker,worker_thread,timer,timer_thread

class SprayGUI(QDialog):
    def __init__(self, parent=None):
        super(SprayGUI,self).__init__(parent)

        self.originalPalette = QApplication.palette()

        self.robot_ip='127.0.0.1'
        self.realrobot=False
        ###tesseract visualizer
        try:
            self.tes_env=Tess_Env('config/urdf/')
        except Exception as e:
            print(e)
            print("Start without tesseract visualizer")
            self.tes_env=None

        # robot box
        robot_list=['','FANUC_m710ic','FANUC_m900ia','FANUC_m10ia','FANUC_lrmate200id','ABB_1200_5_90','ABB_6640_180_255']
        self.robotComBox1=QComboBox()
        self.robotComBox1.addItems(robot_list)
        self.robotComBox1.currentTextChanged.connect(self.robot1_change)
        robotlabel1=QLabel("Robot1:")
        robotlabel1.setBuddy(self.robotComBox1)
        self.robotComBox2=QComboBox()
        self.robotComBox2.addItems(robot_list)
        self.robotComBox2.currentTextChanged.connect(self.robot2_change)
        robotlabel2=QLabel("Robot2:")
        robotlabel2.setBuddy(self.robotComBox2)

        ###reset visualization button
        vis_reset_button=QPushButton('Reset Visualization')
        vis_reset_button.setDefault(False)
        vis_reset_button.clicked.connect(self.reset_visualization)

        ###ROBOT CONTROLLER IP
        IPlabel=QLabel("ROBOT IP:")
        self.robot_ip_box=QLineEdit('127.0.0.1')
        self.robot_ip_box.setFixedWidth(100)

        IPlabel.setBuddy(self.robot_ip_box)
        ip_set_button=QPushButton('Set IP')
        ip_set_button.setDefault(False)
        ip_set_button.clicked.connect(self.read_ip)

        ### Dual Robot Activation
        self.dualRobot_box=QCheckBox("Dual Robot")
        self.dualRobot_box.setChecked(False)
        self.dualRobot_box.stateChanged.connect(self.dualRobotActFunc)

        ### real Robot Activation
        self.realrobot_button= QPushButton('Real Robot')
        self.realrobot_button.setDefault(False)
        self.realrobot_button.clicked.connect(self.changeColor)

        ## Redundancy Resolution Box
        self.redundancyResLeft()
        ## Motion Program Generation Box
        self.motionProgGenMid()
        ## Motion Program Update Box
        self.motionProgUpdateRight()

        ## toplayout
        self.toplayout=QHBoxLayout()
        self.toplayout.addWidget(robotlabel1)
        self.toplayout.addWidget(self.robotComBox1)
        self.toplayout.addStretch(1)
        self.toplayout.addWidget(robotlabel2)
        self.toplayout.addWidget(self.robotComBox2)
        self.toplayout.addWidget(vis_reset_button)
        self.toplayout.addWidget(IPlabel)
        self.toplayout.addWidget(self.robot_ip_box)
        self.toplayout.addWidget(ip_set_button)
        self.toplayout.addWidget(self.dualRobot_box)
        self.toplayout.addWidget(self.realrobot_button)

        ## main layout
        self.mainLayout = QGridLayout()
        self.mainLayout.addLayout(self.toplayout,0,0,1,2)
        self.mainLayout.addWidget(self.redResLeftBox,1,0)
        self.mainLayout.addWidget(self.moProgGenMidBox,1,1)
        self.mainLayout.addWidget(self.moProgUpRightBox,1,2)
        self.setLayout(self.mainLayout)

        self.setWindowTitle('ROBOT PATH OPTIMIZATION GUI')
        self.changeStyle('Windows')

        ###### spray functions variable
        self.curve_pathname=None
        self.curve_filename=None
        self.curvejs1_filename=None
        self.curvejs2_filename=None
        self.curvejs_pathname=None
        self.des_curve_pathname=None
        self.des_curve_filename=None
        self.des_curvejs1_filename=None
        self.des_curvejs_pathname=None
        self.cmd_filename=None
        self.cmd_pathname=None
        self.robot1=None
        self.robot1_name=None
        self.robot2=None
        self.robot2_name=None

    def changeColor(self):
        # if button is checked
        if not self.realrobot:
            # setting background color to light-blue
            self.realrobot_button.setStyleSheet("background-color : lightgreen")
            self.realrobot=True
        # if it is unchecked
        else:
            # set background color back to light-grey
            self.realrobot_button.setStyleSheet("background-color : lightgray")
            self.realrobot=False
    def changeStyle(self, styleName):
        QApplication.setStyle(QStyleFactory.create(styleName))
        self.changePalette()
    def changePalette(self):
        QApplication.setPalette(self.originalPalette)
    def robot1_change(self,robot1_choose):
        
        if robot1_choose == '':
            return
        else:
            self.robot1=robot_obj(robot1_choose,'config/'+robot1_choose+'_robot_default_config.yml',tool_file_path='config/paintgun.csv',d=50,acc_dict_path='config/'+robot1_choose+'_acc.pickle')
            if 'ABB' in robot1_choose:
                self.robot1MotionSend=MotionSendABB             ###TODO: add realrobot argument (IP, repeatibility)
            elif 'FANUC' in robot1_choose:
                self.robot1=robot_obj(robot1_choose,'config/'+robot1_choose+'_robot_default_config.yml',tool_file_path='config/paintgun.csv',d=50,acc_dict_path='config/'+robot1_choose+'_acc_compensate.pickle',j_compensation=[1,1,-1,-1,-1,-1])
                self.robot1MotionSend=MotionSendFANUC             ###TODO: add tool from robot def (FANUC)           
                # self.robot1=m10ia(d=50)
        self.robot1_name=robot1_choose

        if self.tes_env is not None:
            self.tes_env.update_pose(self.robot1_name,np.eye(4))

    def robot2_change(self,robot2_choose):

        self.robot2_name=robot2_choose
    
    def showdialog(self,message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)

        msg.setText(message)
        # msg.setInformativeText("This is additional information")
        # msg.setWindowTitle("MessageBox demo")
        # msg.setDetailedText("The details are as follows:")
        msg.setStandardButtons(QMessageBox.Ok)# | QMessageBox.Cancel)
        msg.buttonClicked.connect(msgbtn)

        retval = msg.exec_()


    def dualRobotActFunc(self):

        ########## update all box
        ## Redundancy Resolution Box
        self.redundancyResLeft()
        ## Motion Program Generation Box
        self.motionProgGenMid()
        ## Motion Program Update Box
        self.motionProgUpdateRight()
        ############################

        ## main layout
        # self.mainLayout = QGridLayout()
        # self.mainLayout.addLayout(self.toplayout,0,0,1,2)
        # self.mainLayout.addWidget(self.redResLeftBox,1,0)
        # self.mainLayout.addWidget(self.moProgGenMidBox,1,1)
        # self.mainLayout.addWidget(self.moProgUpRightBox,1,2)
        # self.setLayout(self.mainLayout)
        leftitem = self.mainLayout.itemAtPosition(1,0)
        self.mainLayout.removeItem(leftitem)
        leftitem.widget().deleteLater()
        self.mainLayout.addWidget(self.redResLeftBox,1,0)
        miditem = self.mainLayout.itemAtPosition(1,1)
        self.mainLayout.removeItem(miditem)
        miditem.widget().deleteLater()
        self.mainLayout.addWidget(self.moProgGenMidBox,1,1)
        rightitem = self.mainLayout.itemAtPosition(1,2)
        self.mainLayout.removeItem(rightitem)
        rightitem.widget().deleteLater()
        self.mainLayout.addWidget(self.moProgUpRightBox,1,2)
    
    def redundancyResLeft(self):

        # Group Box
        self.redResLeftBox=QGroupBox("Redundancy Resolution")
        
        # add widgets
        filebutton=QPushButton('Open Curve File')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCurveFile)
        self.curve_filenametext=QLabel('(Please add curve file)')

        if not self.dualRobot_box.isChecked():
            self.redres_baseline_runButton=QPushButton("Run Baseline")
            self.redres_baseline_runButton.setDefault(True)
            self.redres_baseline_runButton.clicked.connect(self.run_RedundancyResolution_baseline)

        self.v_cmd_box = QSpinBox(self.redResLeftBox)
        self.v_cmd_box.setMinimum(100)
        self.v_cmd_box.setMaximum(2000)
        self.v_cmd_box.setValue(500)
        self.v_cmd_box.setSingleStep(50)
        v_cmd_qt=QLabel('Aggressive Velocity')
        v_cmd_qt.setBuddy(self.v_cmd_box)

        ##### transformation param
        if self.dualRobot_box.isChecked():
            baseTransformHeadsup=QLabel('Enter Robot2 Base Transformation Initial Guess')
            baseTransLayout=QHBoxLayout()
            # baseRotLayout=QHBoxLayout()
            self.r2_basePx_box=QLineEdit('1000')
            r2_basePx_label=QLabel('Px')
            r2_basePx_label.setBuddy(self.r2_basePx_box)
            self.r2_basePy_box=QLineEdit('0')
            r2_basePy_label=QLabel('Py')
            r2_basePy_label.setBuddy(self.r2_basePy_box)
            self.r2_basePz_box=QLineEdit('0')
            r2_basePz_label=QLabel('Pz')
            r2_basePz_label.setBuddy(self.r2_basePz_box)
            self.r2_baseRz_box=QLineEdit('180')
            r2_baseRz_label=QLabel('Rz')
            r2_baseRz_label.setBuddy(self.r2_baseRz_box)
            baseTransLayout.addWidget(r2_basePx_label)
            baseTransLayout.addWidget(self.r2_basePx_box)
            baseTransLayout.addWidget(r2_basePy_label)
            baseTransLayout.addWidget(self.r2_basePy_box)
            baseTransLayout.addWidget(r2_basePz_label)
            baseTransLayout.addWidget(self.r2_basePz_box)
            baseTransLayout.addWidget(r2_baseRz_label)
            baseTransLayout.addWidget(self.r2_baseRz_box)

            qinit2Headsup=QLabel('Enter Robot2 Joint Initial Guess (degree)')
            qinit2Layout=QHBoxLayout()
            self.r2init_boxes=[]
            r2init_labels=[]
            for j in range(6):
                self.r2init_boxes.append(QLineEdit())
                r2init_labels.append(QLabel('q'+str(j+1)))
                r2init_labels[-1].setBuddy(self.r2init_boxes[-1])
                qinit2Layout.addWidget(r2init_labels[-1])
                qinit2Layout.addWidget(self.r2init_boxes[-1])
            self.r2init_boxes[0].setText('0')
            self.r2init_boxes[1].setText('10')
            self.r2init_boxes[2].setText('10')
            self.r2init_boxes[3].setText('0')
            self.r2init_boxes[4].setText('-30')
            self.r2init_boxes[5].setText('90')

            self.opt_base_box=QCheckBox("Optimize Base")
            self.opt_base_box.setChecked(True)
        ##########################

        self.redres_diffevo_runButton=QPushButton("Run DiffEvo (>1day, needs to run baseline first)")
        self.redres_diffevo_runButton.setDefault(True)

        self.redres_diffevo_runButton.clicked.connect(self.run_RedundancyResolution_diffevo)

        self.run1_result=QLabel('')

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(filebutton)
        layout.addWidget(self.curve_filenametext)

        if not self.dualRobot_box.isChecked():
            layout.addWidget(self.redres_baseline_runButton)
        layout.addWidget(v_cmd_qt)
        layout.addWidget(self.v_cmd_box)

        if self.dualRobot_box.isChecked():
            layout.addWidget(baseTransformHeadsup)
            layout.addLayout(baseTransLayout)
            # layout.addLayout(baseRotLayout)
            layout.addWidget(qinit2Headsup)
            layout.addLayout(qinit2Layout)
            layout.addWidget(self.opt_base_box)

        layout.addWidget(self.redres_diffevo_runButton)
        layout.addWidget(self.run1_result)
        layout.addStretch(1)
        self.redResLeftBox.setLayout(layout)

    def reset_visualization(self):

        if self.tes_env is None:
            return
        ###TODO: FIX QUICK TESSERACT RESET
        # self.tes_env=Tess_Env('config/urdf/')
        H=np.eye(4)
        H[:-1,-1]=100*np.ones(3)
        self.tes_env.update_pose('ABB_6640_180_255',H)
        self.tes_env.update_pose('FANUC_lrmate200id',H)
        self.tes_env.update_pose('ABB_1200_5_90',H)
        self.tes_env.update_pose('FANUC_m10ia',H)
        self.tes_env.attach_part('curve_1','world')
        self.tes_env.attach_part('curve_2','world')


    def read_ip(self):
        self.robot_ip= self.robot_ip_box.text()
        ret = ping(self.robot_ip)
        if ret:
            self.showdialog('IP Address Set!')
        else:
            self.showdialog('IP Address Not Reachable, Please Check Connection!')

    def readCurveFile(self):
        
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        dlg.setFilter(QDir.Files)

        if dlg.exec_():
            filenames = dlg.selectedFiles()
            self.curve_filename = filenames[0]
            self.curve_pathname = os.path.dirname(self.curve_filename)
            self.curve_filenametext.setText(self.curve_filename)
    
    def run_RedundancyResolution_baseline(self):

        if self.robot1 is None:
            self.run1_result.setText("Robot1 not yet choosed.")
            return
        if self.curve_filename is None:
            self.run1_result.setText("Curve File not yet choosed.")
            return
        
        self.run1_result.setText('Solving Redundancy Resolution')

        # qthread for redundancy resolution
        self.redres_thread=QThread()
        self.redres_timer_thread=QThread()
        self.redres_worker=Worker(redundancy_resolution_baseline,self.curve_filename,self.robot1)
        self.redres_timer=Timer()
        

        self.redres_worker,self.redres_thread,self.redres_timer,self.redres_timer_thread=\
            setup_worker_timer(self.redres_worker,self.redres_thread,self.redres_timer,self.redres_timer_thread,\
                self.prog_RedundancyResolution,self.res_RedundancyResolution_baseline)

        ## edit interface
        self.redres_baseline_runButton.setEnabled(False)
        
        ## final result setup
        self.redres_thread.finished.connect(lambda: self.redres_baseline_runButton.setEnabled(True))

        ## start thread
        self.redres_thread.start()
        self.redres_timer_thread.start()


    def run_RedundancyResolution_diffevo(self):

        if self.robot1 is None:
            self.run1_result.setText("Robot1 not yet choosed.")
            return
        if self.curve_filename is None:
            self.run1_result.setText("Curve File not yet choosed.")
            return
        
        v_cmd=self.v_cmd_box.value()
        self.run1_result.setText('Solving Redundancy Resolution')

        # qthread for redundancy resolution
        self.redres_thread=QThread()
        self.redres_timer_thread=QThread()
        self.redres_timer=Timer()
        
        if not self.dualRobot_box.isChecked():
            baseline_pose_filename=os.path.dirname(self.curve_filename)+'/'+self.robot1_name+'/baseline/curve_pose.csv'
            self.redres_worker=Worker(redundancy_resolution_diffevo,self.curve_filename,baseline_pose_filename,self.robot1,v_cmd)
        else:
            curve_dir=os.path.dirname(self.curve_filename)
            if self.robot2_name is None:
                self.run1_result.setText("Robot2 not yet choosed.")
                return
            ### set robot2
            try:
                if 'ABB' in self.robot2_name:
                    self.robot2=robot_obj(self.robot2_name,'config/'+self.robot2_name+'_robot_default_config.yml',tool_file_path=self.robot1_name+'_'+self.robot2_name+'/tcp_workpiece.csv',acc_dict_path='config/'+self.robot2_name+'_acc.pickle')
                elif 'FANUC' in self.robot2_name:
                    self.robot2=robot_obj(self.robot2_name,'config/'+self.robot2_name+'_robot_default_config.yml',tool_file_path=self.robot1_name+'_'+self.robot2_name+'/tcp_workpiece.csv',acc_dict_path='config/'+self.robot2_name+'_acc_compensate.pickle',j_compensation=[1,1,-1,-1,-1,-1])
            except Exception as e:
                print(e)
                return
            #################################
            base_T=np.eye(4)
            base_T[:3,:3]=Rz(np.radians(float(self.r2_baseRz_box.text())))
            base_T[:3,-1]=np.array([float(self.r2_basePx_box.text()),float(self.r2_basePy_box.text()),float(self.r2_basePz_box.text())])
            q_init2=[]
            for j in range(6):
                q_init2.append(float(self.r2init_boxes[j].text()))
            q_init2=np.radians(q_init2)
            self.redres_worker=Worker(redundancy_resolution_diffevo_dual,self.curve_filename,base_T,self.robot1,self.robot2,q_init2,int(v_cmd),self.opt_base_box.isChecked())
        self.redres_worker,self.redres_thread,self.redres_timer,self.redres_timer_thread=\
            setup_worker_timer(self.redres_worker,self.redres_thread,self.redres_timer,self.redres_timer_thread,\
                self.prog_RedundancyResolution,self.res_RedundancyResolution_diffevo)

        ## edit interface
        self.redres_diffevo_runButton.setEnabled(False)
        self.dualRobot_box.setEnabled(False)
        
        ## final result setup
        self.redres_thread.finished.connect(lambda: self.redres_diffevo_runButton.setEnabled(True))
        self.redres_thread.finished.connect(lambda: self.dualRobot_box.setEnabled(True))

        ## start thread
        self.redres_thread.start()
        self.redres_timer_thread.start()
    
    def prog_RedundancyResolution(self,n):
        
        self.run1_result.setText('Solving Redundancy Resolution. Time: '+time.strftime("%H:%M:%S", time.gmtime(n)))
    
    def res_RedundancyResolution_baseline(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run1_result.setText('Redundancy Resolution failed because of errors. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        curve_base,curve_normal_base,curve_js,H,run_duration=result

        robot_path=self.curve_pathname+'/'+self.robot1_name+'/'
        Path(robot_path).mkdir(exist_ok=True)
        save_filepath=self.curve_pathname+'/'+self.robot1_name+'/baseline/'
        Path(save_filepath).mkdir(exist_ok=True)

        ###save file
        df=DataFrame({'x':curve_base[:,0],'y':curve_base[:,1], 'z':curve_base[:,2],'x_dir':curve_normal_base[:,0],'y_dir':curve_normal_base[:,1], 'z_dir':curve_normal_base[:,2]})
        df.to_csv(save_filepath+'Curve_in_base_frame.csv',header=False,index=False)
        
        np.savetxt(save_filepath+'curve_pose.csv',H,delimiter=',')

        if len(curve_js) > 0:
            DataFrame(curve_js).to_csv(save_filepath+'Curve_js.csv',header=False,index=False)
            self.run1_result.setText('Redundancy Resolution Solved\nFile Path:\n'+save_filepath+'\nTotal Time: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
        else:
            self.run1_result.setText('Redundancy Resolution. No JS Solution. \nFile Path:\n'+save_filepath+'\nTotal Time: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))

    def res_RedundancyResolution_diffevo(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run1_result.setText('Redundancy Resolution failed because of errors. Time:',time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        if not self.dualRobot_box.isChecked():
            curve_base,curve_normal_base,curve_js,H,run_duration=result
            save_filepath=self.curve_pathname+'/'+self.robot1_name+'/diffevo/'
            Path(save_filepath).mkdir(exist_ok=True)
            ###save file
            df=DataFrame({'x':curve_base[:,0],'y':curve_base[:,1], 'z':curve_base[:,2],'x_dir':curve_normal_base[:,0],'y_dir':curve_normal_base[:,1], 'z_dir':curve_normal_base[:,2]})
            df.to_csv(save_filepath+'Curve_in_base_frame.csv',header=False,index=False)
            np.savetxt(save_filepath+'curve_pose.csv',H,delimiter=',')
            if len(curve_js) > 0:
                DataFrame(curve_js).to_csv(save_filepath+'Curve_js.csv',header=False,index=False)
                self.run1_result.setText('Redundancy Resolution Solved\nFile Path:\n'+save_filepath+'\nTotal Time: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            else:
                self.run1_result.setText('Redundancy Resolution. No JS Solution. \nFile Path:\n'+save_filepath+'\nTotal Time: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
        else:
            curve_js1,curve_js2,robot2_base,run_duration=result
            save_filepath=self.curve_pathname+'/'+self.robot1_name+'_'+self.robot2_name+'/diffevo/'
            Path(self.curve_pathname+'/'+self.robot1_name+'_'+self.robot2_name).mkdir(exist_ok=True)
            Path(save_filepath).mkdir(exist_ok=True)
            DataFrame(curve_js1).to_csv(save_filepath+'Curve_js1.csv',header=False,index=False)
            DataFrame(curve_js2).to_csv(save_filepath+'Curve_js2.csv',header=False,index=False)
            np.savetxt(save_filepath+'base.csv',robot2_base,delimiter=',')

    def motionProgGenMid(self):

        # Group Box
        self.moProgGenMidBox=QGroupBox('Motion Program Generation')

        # add button
        # filebutton=QPushButton('Open Curve Js File')
        #### open solution file
        filebutton=QPushButton('Open Solution Directory')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCurveJsFile)
        self.curvejs_solutionDirtext=QLabel('(Please select solution directory)')
        ########################

        self.total_seg_box = QSpinBox(self.moProgGenMidBox)
        self.total_seg_box.setMinimum(1)
        self.total_seg_box.setMaximum(50000)
        self.total_seg_box.setValue(100)
        seglabel=QLabel('Total Segments:')
        seglabel.setBuddy(self.total_seg_box)
        self.moprog_runButton_baseline=QPushButton("Run Baseline")
        self.moprog_runButton_baseline.setDefault(True)
        self.moprog_runButton_baseline.clicked.connect(self.run_MotionProgGeneration_baseline)

        self.greedy_thresh_box = QDoubleSpinBox(self.moProgGenMidBox)
        self.greedy_thresh_box.setMinimum(0.01)
        self.greedy_thresh_box.setMaximum(0.5)
        self.greedy_thresh_box.setValue(0.1)
        self.greedy_thresh_box.setSingleStep(0.02)
        greedy_thresh=QLabel('Threshold:')
        greedy_thresh.setBuddy(self.greedy_thresh_box)
        self.moprog_runButton_greedy=QPushButton("Run Greedy")
        self.moprog_runButton_greedy.setDefault(True)
        self.moprog_runButton_greedy.clicked.connect(self.run_MotionProgGeneration_greedy)

        self.run2_result=QLabel('')

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(filebutton)
        layout.addWidget(self.curvejs_solutionDirtext)
        layout.addWidget(seglabel)
        layout.addWidget(self.total_seg_box)
        layout.addWidget(self.moprog_runButton_baseline)
        layout.addWidget(self.greedy_thresh_box)
        layout.addWidget(self.moprog_runButton_greedy)
        layout.addWidget(self.run2_result)
        layout.addStretch(1)
        self.moProgGenMidBox.setLayout(layout)
    
    def readCurveJsFile(self):
        
        try:
            solution_dir = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
            self.curvejs_pathname = solution_dir
            self.curvejs1_filename = solution_dir+'/Curve_js1.csv'
            if self.dualRobot_box.isChecked():
                self.curvejs2_filename = solution_dir+'/Curve_js2.csv'
            self.curvejs_solutionDirtext.setText(solution_dir)
        except Exception as e:
            print(e)

    def run_MotionProgGeneration_baseline(self):

        if self.robot1 is None:
            self.run2_result.setText("Robot1 not yet choosed.")
            return
        if self.curvejs1_filename is None:
            self.run2_result.setText("Solution folder not chosen or Curve_js1 not exists")
            return

        self.run2_result.setText('Generating Motion Program')
        total_seg=int(self.total_seg_box.value())

        # qthread for redundancy resolution
        self.moprog_thread=QThread()
        self.moprog_timer_thread=QThread()
        self.moprog_timer=Timer()

        if not self.dualRobot_box.isChecked():
            self.moprog_worker=Worker(motion_program_generation_baseline,self.curvejs1_filename,self.robot1,total_seg)
        else:
            if self.robot2_name is None:
                self.run2_result.setText("Robot2 not yet choosed.")
                return
            if self.curvejs2_filename is None:
                self.run2_result.setText("Solution folder not chosen or Curve_js2 not exists")
                return
            ### set robot2
            if 'ABB' in self.robot2_name:
                self.robot2=robot_obj(self.robot2_name,'config/'+self.robot2_name+'_robot_default_config.yml',tool_file_path=self.curvejs_pathname+'/../tcp_workpiece.csv',base_transformation_file=self.curvejs_pathname+'/base.csv',acc_dict_path='config/'+self.robot2_name+'_acc.pickle')
            elif 'FANUC' in self.robot2_name:
                self.robot2=robot_obj(self.robot2_name,'config/'+self.robot2_name+'_robot_default_config.yml',tool_file_path=self.curvejs_pathname+'/../tcp_workpiece.csv',base_transformation_file=self.curvejs_pathname+'/base.csv',acc_dict_path='config/'+self.robot2_name+'_acc_compensate.pickle',j_compensation=[1,1,-1,-1,-1,-1])
            #################################
            self.moprog_worker=Worker(motion_program_generation_baseline_dual,self.curvejs1_filename,self.robot1,self.curvejs2_filename,self.robot2,total_seg)

        self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread=\
            setup_worker_timer(self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread,\
                self.prog_MotionProgGeneration,self.res_MotionProgGeneration_baseline)

        ## edit interface
        self.moprog_runButton_baseline.setEnabled(False)
        self.total_seg_box.setEnabled(False)
        self.dualRobot_box.setEnabled(False)
        ## final result setup
        self.moprog_thread.finished.connect(lambda: self.moprog_runButton_baseline.setEnabled(True))
        self.moprog_thread.finished.connect(lambda: self.total_seg_box.setEnabled(True))
        self.moprog_thread.finished.connect(lambda: self.dualRobot_box.setEnabled(True))

        ## start thread
        self.moprog_timer_thread.start()
        self.moprog_thread.start()

    def run_MotionProgGeneration_greedy(self):

        if self.robot1 is None:
            self.run2_result.setText("Robot1 not yet choosed.")
            return
        if self.curvejs1_filename is None:
            self.run2_result.setText("Solution folder not chosen or Curve_js1 not exists")
            return

        self.run2_result.setText('Generating Motion Program')
        greedy_thresh=self.greedy_thresh_box.value()

        # qthread for redundancy resolution
        self.moprog_thread=QThread()
        self.moprog_timer_thread=QThread()
        self.moprog_timer=Timer()

        if not self.dualRobot_box.isChecked():
            self.moprog_worker=Worker(motion_program_generation_greedy,self.curvejs1_filename,self.robot1,greedy_thresh)
        else:
            if self.robot2_name is None:
                self.run2_result.setText("Robot2 not yet choosed.")
                return
            if self.curvejs2_filename is None:
                self.run2_result.setText("Solution folder not chosen or Curve_js2 not exists")
                return
            ### set robot2
            if 'ABB' in self.robot2_name:
                self.robot2=robot_obj(self.robot2_name,'config/'+self.robot2_name+'_robot_default_config.yml',tool_file_path=self.curvejs_pathname+'/../tcp_workpiece.csv',base_transformation_file=self.curvejs_pathname+'/base.csv',acc_dict_path='config/'+self.robot2_name+'_acc.pickle')
            elif 'FANUC' in self.robot2_name:
                self.robot2=robot_obj(self.robot2_name,'config/'+self.robot2_name+'_robot_default_config.yml',tool_file_path=self.curvejs_pathname+'/../tcp_workpiece.csv',base_transformation_file=self.curvejs_pathname+'/base.csv',acc_dict_path='config/'+self.robot2_name+'_acc_compensate.pickle',j_compensation=[1,1,-1,-1,-1,-1])
            #################################
            self.moprog_worker=Worker(motion_program_generation_greedy_dual,self.curvejs1_filename,self.robot1,self.curvejs2_filename,self.robot2,greedy_thresh)

        self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread=\
            setup_worker_timer(self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread,\
                self.prog_MotionProgGeneration,self.res_MotionProgGeneration_greedy)

        ## edit interface
        self.moprog_runButton_greedy.setEnabled(False)
        self.greedy_thresh_box.setEnabled(False)
        self.dualRobot_box.setEnabled(False)
        ## final result setup
        self.moprog_thread.finished.connect(lambda: self.moprog_runButton_greedy.setEnabled(True))
        self.moprog_thread.finished.connect(lambda: self.total_seg_box.setEnabled(True))
        self.moprog_thread.finished.connect(lambda: self.dualRobot_box.setEnabled(True))

        ## start thread
        self.moprog_timer_thread.start()
        self.moprog_thread.start()
    
    def prog_MotionProgGeneration(self,n):
        
        self.run2_result.setText('Generating Motion Program. Time: '+time.strftime("%H:%M:%S", time.gmtime(n)))

    def res_MotionProgGeneration_baseline(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run2_result.setText('Motion Program Generation Failed. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        if not self.dualRobot_box.isChecked():
            breakpoints,primitives,q_bp,p_bp,run_duration=result
            save_filepath=self.curvejs_pathname+'/'+str(int(self.total_seg_box.value()))+'L/'
            Path(save_filepath).mkdir(exist_ok=True)
            # save motion program commands
            df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
            df.to_csv(save_filepath+'command.csv',header=True,index=False)
        else:
            breakpoints1,primitives1,q_bp1,p_bp1,breakpoints2,primitives2,q_bp2,p_bp2,run_duration=result
            save_filepath=self.curvejs_pathname+'/'+str(int(self.total_seg_box.value()))+'L_dual/'
            Path(save_filepath).mkdir(exist_ok=True)
            # save motion program commands
            df=DataFrame({'breakpoints':breakpoints1,'primitives':primitives1,'p_bp':p_bp1,'q_bp':q_bp1})
            df.to_csv(save_filepath+'command1.csv',header=True,index=False)
            df=DataFrame({'breakpoints':breakpoints2,'primitives':primitives2,'p_bp':p_bp2,'q_bp':q_bp2})
            df.to_csv(save_filepath+'command2.csv',header=True,index=False)

        self.run2_result.setText('Motion Program Generated\nTime: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))

    def res_MotionProgGeneration_greedy(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run2_result.setText('Motion Program Generation Failed. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        if not self.dualRobot_box.isChecked():
            breakpoints,primitives,q_bp,p_bp,run_duration=result
            save_filepath=self.curvejs_pathname+'/greedy'+str(self.greedy_thresh_box.value())+'/'
            Path(save_filepath).mkdir(exist_ok=True)
            # save motion program commands
            df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
            df.to_csv(save_filepath+'command.csv',header=True,index=False)
        else:
            breakpoints1,primitives1,q_bp1,p_bp1,breakpoints2,primitives2,q_bp2,p_bp2,run_duration=result
            save_filepath=self.curvejs_pathname+'/greedy'+str(self.greedy_thresh_box.value())+'_dual/'
            Path(save_filepath).mkdir(exist_ok=True)
            # save motion program commands
            df=DataFrame({'breakpoints':breakpoints1,'primitives':primitives1,'p_bp':p_bp1,'q_bp':q_bp1})
            df.to_csv(save_filepath+'command1.csv',header=True,index=False)
            df=DataFrame({'breakpoints':breakpoints2,'primitives':primitives2,'p_bp':p_bp2,'q_bp':q_bp2})
            df.to_csv(save_filepath+'command2.csv',header=True,index=False)

        self.run2_result.setText('Motion Program Generated\nTime: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
        
    def motionProgUpdateRight(self):
        
        self.moProgUpRightBox=QGroupBox('Motion Program Update')

        # add button
        solution_button=QPushButton('Open Solution Directory')
        solution_button.setDefault(True)
        solution_button.clicked.connect(self.read_Solution)
        self.solutionDirtext=QLabel('(Please select solution directory)')

        filebutton=QPushButton('Open Command Directory')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCmdFile)
        self.cmd_filenametext=QLabel('(Please select motion command directory)')

        # box for vel
        self.vel_box = QSpinBox()
        self.vel_box.setMinimum(1)
        self.vel_box.setMaximum(2000)
        self.vel_box.setValue(500)
        self.vel_box.setSingleStep(10)
        vellabel=QLabel('Robot Velocity:')
        vellabel.setBuddy(self.vel_box)

        # add box for tolerance setup
        self.error_box = QDoubleSpinBox()
        self.error_box.setMinimum(0.1)
        self.error_box.setMaximum(5)
        self.error_box.setValue(0.5)
        errorlabel=QLabel('Error (mm):')
        errorlabel.setBuddy(self.error_box)

        self.ang_error_box = QDoubleSpinBox()
        self.ang_error_box.setMinimum(0.5)
        self.ang_error_box.setMaximum(10)
        self.ang_error_box.setValue(3)
        angerrorlabel=QLabel('Angular Error (deg):')
        angerrorlabel.setBuddy(self.ang_error_box)

        self.vel_std_box = QDoubleSpinBox()
        self.vel_std_box.setMinimum(1)
        self.vel_std_box.setMaximum(20)
        self.vel_std_box.setValue(5)
        velstdlabel=QLabel('Velocity Std (%):')
        velstdlabel.setBuddy(self.vel_std_box)

        self.extend_start_box= QDoubleSpinBox()
        self.extend_start_box.setMinimum(1)
        self.extend_start_box.setMaximum(500)
        self.extend_start_box.setValue(100)
        extend_start_label=QLabel('Ext Start:')
        extend_start_label.setBuddy(self.extend_start_box)

        self.extend_end_box= QDoubleSpinBox()
        self.extend_end_box.setMinimum(1)
        self.extend_end_box.setMaximum(500)
        self.extend_end_box.setValue(100)
        extend_end_label=QLabel('Ext End:')
        extend_end_label.setBuddy(self.extend_end_box)

        self.visual_runButton=QPushButton("Run Visualization")
        self.visual_runButton.setDefault(True)
        self.visual_runButton.clicked.connect(self.run_Visualization)

        ### for FANUC utool
        if self.dualRobot_box.isChecked():
            # if "FANUC" in self.robot1.robot_name:
            utool2_layout=QHBoxLayout()
            utool2_label=QLabel('Robot2 Utool')
            self.utool2_box=QSpinBox()
            self.utool2_box.setMinimum(1)
            self.utool2_box.setMaximum(20)
            self.utool2_box.setValue(2)
            self.utool2_box.setSingleStep(1)
            utool2_label.setBuddy(self.utool2_box)
            utool2_layout.addWidget(utool2_label)
            utool2_layout.addWidget(self.utool2_box)

        self.moupdate_runButton=QPushButton("Run Motion Update")
        self.moupdate_runButton.setDefault(True)
        self.moupdate_runButton.clicked.connect(self.run_MotionProgUpdate)
        self.run4_result=QLabel('')
        self.run4_result_img=QLabel('')

        # boxes
        vellayout=QHBoxLayout()
        vellayout.addWidget(vellabel)
        vellayout.addWidget(self.vel_box)
        vellayout.addStretch(1)

        tollayout=QHBoxLayout()
        tollayout.addWidget(errorlabel)
        tollayout.addWidget(self.error_box)
        tollayout.addWidget(angerrorlabel)
        tollayout.addWidget(self.ang_error_box)
        tollayout.addWidget(velstdlabel)
        tollayout.addWidget(self.vel_std_box)

        extendlayout=QHBoxLayout()
        extendlayout.addWidget(extend_start_label)
        extendlayout.addWidget(self.extend_start_box)
        extendlayout.addWidget(extend_end_label)
        extendlayout.addWidget(self.extend_end_box)

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(solution_button)
        layout.addWidget(self.solutionDirtext)
        
        layout.addWidget(filebutton)
        layout.addWidget(self.cmd_filenametext)
        layout.addLayout(vellayout)
        layout.addLayout(tollayout)
        layout.addLayout(extendlayout)
        if self.dualRobot_box.isChecked():
            layout.addLayout(utool2_layout)
        layout.addWidget(self.visual_runButton)
        layout.addWidget(self.moupdate_runButton)
        layout.addWidget(self.run4_result)
        layout.addWidget(self.run4_result_img)
        layout.addStretch(1)
        self.moProgUpRightBox.setLayout(layout)
    
    def read_Solution(self):

        try:
            ###TODO: warning if not a valid solution directory
            solution_dir = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
            #read part name
            curve_stringidx1=solution_dir.index('data/')
            part_name=solution_dir[curve_stringidx1+5:]
            curve_stringidx2=part_name.index('/')
            part_name=part_name[:curve_stringidx2]

            ### read curve
            if not self.dualRobot_box.isChecked():
                self.des_curve_filename = solution_dir+'/Curve_in_base_frame.csv'
                self.des_curvejs1_filename = solution_dir+'/Curve_js.csv'
                ### read part/base pose
                self.part_pose=np.loadtxt(solution_dir+'/curve_pose.csv',delimiter=',')
                
                if self.tes_env is not None:             
                    part_pose_m=copy.deepcopy(self.part_pose)
                    part_pose_m[:-1,-1]=part_pose_m[:-1,-1]/1000.
                    self.tes_env.update_pose(part_name,part_pose_m)


            else:
                self.des_curve_filename = solution_dir+'/../../Curve_dense.csv'
                self.base2_pose=np.loadtxt(solution_dir+'/base.csv',delimiter=',')
                self.des_curvejs1_filename = solution_dir+'/Curve_js1.csv'
                self.des_curvejs2_filename = solution_dir+'/Curve_js2.csv'

                if self.tes_env is not None:
                    #update robot2 base
                    base2_pose_m=copy.deepcopy(self.base2_pose)
                    base2_pose_m[:-1,-1]=base2_pose_m[:-1,-1]/1000.
                    self.tes_env.update_pose(self.robot2_name,base2_pose_m)
                    #update robot2 holding part
                    tcp_workpiece=np.loadtxt(solution_dir+'/../tcp_workpiece.csv',delimiter=',')
                    tcp_workpiece[:-1,-1]=tcp_workpiece[:-1,-1]/1000.
                    self.tes_env.attach_part(part_name,self.robot2_name+'_flange',H=tcp_workpiece)
                  
            ### read js1
            self.solutionDirtext.setText(solution_dir)

        except Exception as e:
            self.showdialog(e)

            
    
    def readCmdFile(self):
        
        # dlg = QFileDialog()
        # dlg.setFileMode(QFileDialog.AnyFile)
        # dlg.setFilter(QDir.Files)

        # if dlg.exec_():
        #     filenames = dlg.selectedFiles()
        #     self.cmd_filename = filenames[0]
        #     self.cmd_pathname = os.path.dirname(self.cmd_filename)
        #     self.cmd_filenametext.setText(self.cmd_filename)
        
        try:
            self.cmd_pathname = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
            self.cmd_filenametext.setText(self.cmd_pathname)
        except Exception as e:
            print(e)
    
    def run_Visualization(self):

        if self.tes_env is None:
            self.showdialog("Tesseract Visualizer is not activated")
            return

        if self.robot1 is None:
            self.showdialog("Robot1 not yet choosed.")
            return
        
        if not self.dualRobot_box.isChecked():  #single arm case
            curve_js=np.loadtxt(self.des_curvejs1_filename,delimiter=',')
            self.tes_env.viewer_trajectory(self.robot1_name,curve_js[::100])
        else:                                   #dual arm case
            curve_js1=np.loadtxt(self.des_curvejs1_filename,delimiter=',')
            curve_js2=np.loadtxt(self.des_curvejs2_filename,delimiter=',')
            self.tes_env.viewer_trajectory_dual(self.robot1_name,self.robot2_name,curve_js1[::100],curve_js2[::100])



    def run_MotionProgUpdate(self):

        if self.robot1 is None:
            self.run4_result.setText("Robot1 not yet choosed.")
            return
        if self.cmd_pathname is None:
            self.run4_result.setText("Command file not yet choosed.")
            return
        
        solution_dir=self.solutionDirtext.text()

        vel=int(self.vel_box.value())
        errtol=float(self.error_box.value())
        angerrtol=float(self.ang_error_box.value())
        velstdtol=float(self.vel_std_box.value())
        extstart=float(self.extend_start_box.value())
        extend=float(self.extend_end_box.value())
        self.run4_result.setText('Updating Motion Program')
        self.run4_result_img.setText('')

        ## delete previous tmp result
        if self.realrobot:
            ilc_output='result_speed_'+str(vel)+'_realrobot'
        else:
            ilc_output='result_speed_'+str(vel)
        all_files=os.listdir(self.cmd_pathname)
        if ilc_output in all_files:
            output_dir=self.cmd_pathname+'/'+ilc_output+'/'
            all_files=os.listdir(output_dir)
            if 'final_speed.npy' in all_files:
                os.remove(output_dir+'final_speed.npy')
            if 'final_error.npy' in all_files:
                os.remove(output_dir+'final_error.npy')
            if 'final_ang_error.npy' in all_files:
                os.remove(output_dir+'final_ang_error.npy')
            if 'final_iteration.png' in all_files:
                os.remove(output_dir+'final_iteration.png')
        
        # qthread for motion update
        self.moupdate_thread=QThread()
        self.moupdate_timer_thread=QThread()
        self.moupdate_timer=Timer()

        try:    ###TODO: add error box popup
            if not self.dualRobot_box.isChecked():
                self.moupdate_worker=Worker(motion_program_update,self.cmd_pathname,self.robot1,self.robot_ip,self.robot1MotionSend,vel,self.des_curve_filename,self.des_curvejs1_filename,\
                    errtol,angerrtol,velstdtol,extstart,extend,self.realrobot)
            else:
                if self.robot2_name is None:
                    self.run4_result.setText("Robot2 not yet choosed.")
                    return
                if self.des_curvejs2_filename is None:
                    self.run4_result.setText("Solution folder not chosen or Curve_js2 not exists")
                    return
                ### set robot2
                utool2=None
                if 'ABB' in self.robot2_name:
                    self.robot2=robot_obj(self.robot2_name,'config/'+self.robot2_name+'_robot_default_config.yml',tool_file_path=solution_dir+'/../tcp_workpiece.csv',base_transformation_file=solution_dir+'/base.csv',acc_dict_path='config/'+self.robot2_name+'_acc.pickle')
                elif 'FANUC' in self.robot2_name:
                    self.robot2=robot_obj(self.robot2_name,'config/'+self.robot2_name+'_robot_default_config.yml',tool_file_path=solution_dir+'/../tcp_workpiece.csv',base_transformation_file=solution_dir+'/base.csv',acc_dict_path='config/'+self.robot2_name+'_acc_compensate.pickle',j_compensation=[1,1,-1,-1,-1,-1])
                    utool2=int(self.utool2_box.value())
                #################################
                self.moupdate_worker=Worker(motion_program_update_dual,self.cmd_pathname,self.robot1,self.robot2,self.robot_ip,self.robot1MotionSend,vel,self.des_curve_filename,self.des_curvejs1_filename,self.des_curvejs2_filename,\
                    errtol,angerrtol,velstdtol,extstart,extend,self.realrobot,utool2)
            
            self.moupdate_worker,self.moupdate_thread,self.moupdate_timer,self.moupdate_timer_thread=\
                setup_worker_timer(self.moupdate_worker,self.moupdate_thread,self.moupdate_timer,self.moupdate_timer_thread,\
                    self.prog_MotionProgUpdate,self.res_MotionProgUpdate)
        except:
            traceback.print_exc()
            self.showdialog(traceback.format_exc())

        ## edit interface
        self.moupdate_runButton.setEnabled(False)
        self.vel_box.setEnabled(False)
        self.error_box.setEnabled(False)
        self.ang_error_box.setEnabled(False)
        self.vel_std_box.setEnabled(False)
        ## final result setup
        self.moupdate_thread.finished.connect(lambda: self.moupdate_runButton.setEnabled(True))
        self.moupdate_thread.finished.connect(lambda: self.vel_box.setEnabled(True))
        self.moupdate_thread.finished.connect(lambda: self.error_box.setEnabled(True))
        self.moupdate_thread.finished.connect(lambda: self.ang_error_box.setEnabled(True))
        self.moupdate_thread.finished.connect(lambda: self.vel_std_box.setEnabled(True))

        ## start thread
        self.moupdate_timer_thread.start()
        self.moupdate_thread.start()
    
    def prog_MotionProgUpdate(self,n):

        vel=int(self.vel_box.value())
        if self.realrobot:
            output_dir=self.cmd_pathname+'/result_speed_'+str(vel)+'_realrobot/'
        else:
            output_dir=self.cmd_pathname+'/result_speed_'+str(vel)+'/'
        
        try:
            all_files=os.listdir(output_dir)
        except:
            return

        speed=None
        error=None
        angle_error=None
        if 'final_speed.npy' in all_files:
            speed=np.load(output_dir+'final_speed.npy')
        if 'final_error.npy' in all_files:
            error=np.load(output_dir+'final_error.npy')
        if 'final_ang_error.npy' in all_files:
            angle_error=np.load(output_dir+'final_ang_error.npy')
        if 'final_iteration.png' in all_files:
            pixmap=QPixmap(output_dir+'final_iteration.png')
            self.run4_result_img.setPixmap(pixmap)
        
        result_text='Updating Motion Program. Time:'+time.strftime("%H:%M:%S", time.gmtime(n))
        if speed is not None and error is not None and angle_error is not None:
            result_text+='\nCurrent Max Error:'+str(round(np.max(error),2))+' mm, Max Ang Error:'+str(round(np.max(np.rad2deg(angle_error)),2))+' deg'+\
                '\nAve. Speed:'+str(round(np.mean(speed),2))+' mm/sec, Std. Speed:'+str(round(np.std(speed),2))+' mm/sec, Std/Ave Speed:'+str(round(100*np.std(speed)/np.mean(speed),2))+' %'
        self.run4_result.setText(result_text)

    def res_MotionProgUpdate(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run4_result.setText('Motion Program Update. Failed. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        
        vel=int(self.vel_box.value())

        if self.realrobot:
            ilc_output='/result_speed_'+str(vel)+'_realrobot'
        else:
            ilc_output='/result_speed_'+str(vel)

        # save motion program commands and js execution
        if not self.dualRobot_box.isChecked():
            curve_exe_js,speed,error,angle_error,breakpoints,primitives,q_bp,p_bp,run_duration=result
            df=DataFrame({'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
            df.to_csv(self.cmd_pathname+ilc_output+'/final_command.csv',header=True,index=False)
            DataFrame(curve_exe_js).to_csv(self.cmd_pathname+ilc_output+'/curve_js_exe.csv',header=False,index=False)
        else:
            curve_exe_js1,curve_exe_js2,speed,error,angle_error,breakpoints,primitives1,q_bp1,p_bp1,primitives2,q_bp2,p_bp2,run_duration=result
            df=DataFrame({'primitives':primitives1,'p_bp':p_bp1,'q_bp':q_bp1})
            df.to_csv(self.cmd_pathname+ilc_output+'/final_command1.csv',header=True,index=False)
            df=DataFrame({'primitives':primitives2,'p_bp':p_bp2,'q_bp':q_bp2})
            df.to_csv(self.cmd_pathname+ilc_output+'/final_command2.csv',header=True,index=False)
            DataFrame(curve_exe_js1).to_csv(self.cmd_pathname+ilc_output+'/curve_js1_exe.csv',header=False,index=False)
            DataFrame(curve_exe_js2).to_csv(self.cmd_pathname+ilc_output+'/curve_js2_exe.csv',header=False,index=False)

        result_text='Motion Program Update. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration))+\
            '\nCommand file saved at\n'+self.cmd_pathname+ilc_output
        result_text+='\nMax Error:'+str(round(np.max(error),2))+' mm, Max Ang Error:'+str(round(np.max(angle_error),2))+' deg'+\
            '\nAve. Speed:'+str(round(np.mean(speed),2))+' mm/sec, Std. Speed:'+str(round(np.std(speed),2))+' mm/sec, Std/Ave Speed:'+str(round(100*np.std(speed)/np.mean(speed),2))+' %'
        self.run4_result.setText(result_text)



if __name__ == '__main__':

    app=QApplication(sys.argv)

    window=SprayGUI()
    window.show()
    sys.exit(app.exec())
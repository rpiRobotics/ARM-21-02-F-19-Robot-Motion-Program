from PyQt5.QtCore import QDateTime,Qt,QTimer,QDir,QObject,QRunnable,pyqtSignal,pyqtSlot,QThread
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QDoubleSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget,QFileDialog, QMessageBox)
from PyQt5.QtGui import QPixmap
import sys,os,time,traceback
from pathlib import Path
from pandas import *


from toolbox.robot_def import *
from redundancy_resolution.redundancy_resolution import *
from cmd_gen.cmd_gen import *
from toolbox.abb_utils import *
from toolbox.fanuc_utils import *
from motion_update.motion_update import *
from toolbox.tes_env import *

def msgbtn(i):
    print ("Button pressed is:",i.text())
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

        ###tesseract visualizer
        self.tes_env=Tess_Env('config/urdf/')

        # robot box
        robot_list=['','FANUC_m710ic','FANUC_m900ia','FANUC_m10ia','ABB_1200_5_90','ABB_6640_180_255']
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


        ## Redundancy Resolution Box
        self.redundancyResLeft()
        ## Motion Program Generation Box
        self.motionProgGenMid()
        ## Motion Program Update Box
        self.motionProgUpdateRight()

        ## toplayout
        toplayout=QHBoxLayout()
        toplayout.addWidget(robotlabel1)
        toplayout.addWidget(self.robotComBox1)
        toplayout.addStretch(1)
        toplayout.addWidget(robotlabel2)
        toplayout.addWidget(self.robotComBox2)
        toplayout.addWidget(vis_reset_button)
        toplayout.addWidget(IPlabel)
        toplayout.addWidget(self.robot_ip_box)
        toplayout.addWidget(ip_set_button)


        ## main layout
        mainLayout = QGridLayout()
        mainLayout.addLayout(toplayout,0,0,1,2)
        mainLayout.addWidget(self.redResLeftBox,1,0)
        mainLayout.addWidget(self.moProgGenMidBox,1,1)
        mainLayout.addWidget(self.moProgUpRightBox,1,2)
        self.setLayout(mainLayout)

        self.setWindowTitle('ROBOT PATH OPTIMIZATION GUI')
        self.changeStyle('Windows')

        ###### spray functions variable
        self.curve_pathname=None
        self.curve_filename=None
        self.curvejs_filename=None
        self.curvejs_pathname=None
        self.des_curve_pathname=None
        self.des_curve_filename=None
        self.des_curvejs_filename=None
        self.des_curvejs_pathname=None
        self.cmd_filename=None
        self.cmd_pathname=None
        self.robot1=None
        self.robot1_name=None
        self.robot2=None
        self.robot2_name=None

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
                self.robot1MotionSend=MotionSendFANUC             ###TODO: add tool from robot def (FANUC)           
        self.robot1_name=robot1_choose
        self.tes_env.update_pose(self.robot1_name,np.eye(4))

    def robot2_change(self,robot2_choose):
        print("Robot2 not supported now.")
    
    def showdialog(self,message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)

        msg.setText("This is a message box")
        msg.setInformativeText("This is additional information")
        msg.setWindowTitle("MessageBox demo")
        msg.setDetailedText("The details are as follows:")
        msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        msg.buttonClicked.connect(msgbtn)

        retval = msg.exec_()


    def redundancyResLeft(self):

        # Group Box
        self.redResLeftBox=QGroupBox("Redundancy Resolution")
        
        # add widgets
        filebutton=QPushButton('Open Curve File')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCurveFile)
        self.curve_filenametext=QLabel('(Please add curve file)')
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

        self.redres_diffevo_runButton=QPushButton("Run DiffEvo (>1day, needs to run baseline first)")
        self.redres_diffevo_runButton.setDefault(True)
        self.redres_diffevo_runButton.clicked.connect(self.run_RedundancyResolution_diffevo)

        self.run1_result=QLabel('')

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(filebutton)
        layout.addWidget(self.curve_filenametext)
        layout.addWidget(self.redres_baseline_runButton)
        layout.addWidget(v_cmd_qt)
        layout.addWidget(self.v_cmd_box)
        layout.addWidget(self.redres_diffevo_runButton)
        layout.addWidget(self.run1_result)
        layout.addStretch(1)
        self.redResLeftBox.setLayout(layout)

    def reset_visualization(self):
        H=np.eye(4)
        H[:-1,-1]=100*np.ones(3)
        self.tes_env.update_pose('ABB_6640_180_255',H)
        self.tes_env.update_pose('FANUC_lrmate200id',H)
        self.tes_env.update_pose('ABB_1200_5_90',H)
        self.tes_env.update_pose('FANUC_m10ia',H)
        self.tes_env.update_pose('curve_1',H)
        self.tes_env.update_pose('curve_2',H)

    def read_ip(self):
        self.robot_ip= self.robot_ip_box.text()

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
        baseline_pose_filename=os.path.dirname(self.curve_filename)+'/'+self.robot1_name+'/baseline/curve_pose.csv'
        self.redres_worker=Worker(redundancy_resolution_diffevo,self.curve_filename,baseline_pose_filename,self.robot1,v_cmd)
        self.redres_timer=Timer()
        

        self.redres_worker,self.redres_thread,self.redres_timer,self.redres_timer_thread=\
            setup_worker_timer(self.redres_worker,self.redres_thread,self.redres_timer,self.redres_timer_thread,\
                self.prog_RedundancyResolution,self.res_RedundancyResolution_diffevo)

        ## edit interface
        self.redres_diffevo_runButton.setEnabled(False)
        
        ## final result setup
        self.redres_thread.finished.connect(lambda: self.redres_diffevo_runButton.setEnabled(True))

        ## start thread
        self.redres_thread.start()
        self.redres_timer_thread.start()
    
    def prog_RedundancyResolution(self,n):
        
        self.run1_result.setText('Solving Redundancy Resolution. Time: '+time.strftime("%H:%M:%S", time.gmtime(n)))
    
    def res_RedundancyResolution_baseline(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run1_result.setText('Redundancy Resolution failed because of errors. Time:',time.strftime("%H:%M:%S", time.gmtime(run_duration)))
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



    def motionProgGenMid(self):

        # Group Box
        self.moProgGenMidBox=QGroupBox('Motion Program Generation')

        # add button
        filebutton=QPushButton('Open Curve Js File')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCurveJsFile)
        self.curvejs_filenametext=QLabel('(Please add curve joint space file)')
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
        layout.addWidget(self.curvejs_filenametext)
        layout.addWidget(seglabel)
        layout.addWidget(self.total_seg_box)
        layout.addWidget(self.moprog_runButton_baseline)
        layout.addWidget(self.greedy_thresh_box)
        layout.addWidget(self.moprog_runButton_greedy)
        layout.addWidget(self.run2_result)
        layout.addStretch(1)
        self.moProgGenMidBox.setLayout(layout)
    
    def readCurveJsFile(self):
        
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        dlg.setFilter(QDir.Files)

        if dlg.exec_():
            filenames = dlg.selectedFiles()
            self.curvejs_filename = filenames[0]
            self.curvejs_pathname = os.path.dirname(self.curvejs_filename)
            self.curvejs_filenametext.setText(self.curvejs_filename)
    
    def prog_Visualization(self,n):
        
        self.run3_result.setText('Running Visualization. Time: '+time.strftime("%H:%M:%S", time.gmtime(n)))

    def res_Visualization(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run3_result.setText('Visualization Failed. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        self.run3_result.setText('Visualzation on localhost:8000\nTime: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))


    def run_MotionProgGeneration_baseline(self):

        if self.robot1 is None:
            self.run2_result.setText("Robot1 not yet choosed.")
            return
        if self.curvejs_filename is None:
            self.run2_result.setText("Curve joint space file not yet choosed.")
            return

        self.run2_result.setText('Generating Motion Program')
        total_seg=int(self.total_seg_box.value())

        # qthread for redundancy resolution
        self.moprog_thread=QThread()
        self.moprog_timer_thread=QThread()
        self.moprog_timer=Timer()
        self.moprog_worker=Worker(motion_program_generation_baseline,self.curvejs_filename,self.robot1,total_seg)

        self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread=\
            setup_worker_timer(self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread,\
                self.prog_MotionProgGeneration,self.res_MotionProgGeneration_baseline)

        ## edit interface
        self.moprog_runButton_baseline.setEnabled(False)
        self.total_seg_box.setEnabled(False)
        ## final result setup
        self.moprog_thread.finished.connect(lambda: self.moprog_runButton_baseline.setEnabled(True))
        self.moprog_thread.finished.connect(lambda: self.total_seg_box.setEnabled(True))


        ## start thread
        self.moprog_timer_thread.start()
        self.moprog_thread.start()

    def run_MotionProgGeneration_greedy(self):

        if self.robot1 is None:
            self.run2_result.setText("Robot1 not yet choosed.")
            return
        if self.curvejs_filename is None:
            self.run2_result.setText("Curve joint space file not yet choosed.")
            return

        self.run2_result.setText('Generating Motion Program')
        greedy_thresh=self.greedy_thresh_box.value()

        # qthread for redundancy resolution
        self.moprog_thread=QThread()
        self.moprog_timer_thread=QThread()
        self.moprog_timer=Timer()
        self.moprog_worker=Worker(motion_program_generation_greedy,self.curvejs_filename,self.robot1,greedy_thresh)

        self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread=\
            setup_worker_timer(self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread,\
                self.prog_MotionProgGeneration,self.res_MotionProgGeneration_greedy)

        ## edit interface
        self.moprog_runButton_greedy.setEnabled(False)
        self.greedy_thresh_box.setEnabled(False)
        ## final result setup
        self.moprog_thread.finished.connect(lambda: self.moprog_runButton_greedy.setEnabled(True))
        self.moprog_thread.finished.connect(lambda: self.total_seg_box.setEnabled(True))

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

        breakpoints,primitives,q_bp,p_bp,run_duration=result

        save_filepath=self.curvejs_pathname+'/'+str(int(self.total_seg_box.value()))+'L/'
        Path(save_filepath).mkdir(exist_ok=True)
        # save motion program commands
        df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
        df.to_csv(save_filepath+'command.csv',header=True,index=False)

        self.run2_result.setText('Motion Program Generated\nTime: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))

    def res_MotionProgGeneration_greedy(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run2_result.setText('Motion Program Generation Failed. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        breakpoints,primitives,q_bp,p_bp,run_duration=result

        save_filepath=self.curvejs_pathname+'/greedy'+str(self.greedy_thresh_box.value())+'/'
        Path(save_filepath).mkdir(exist_ok=True)
        # save motion program commands
        df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
        df.to_csv(save_filepath+'command.csv',header=True,index=False)

        self.run2_result.setText('Motion Program Generated\nTime: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))

    def read_Solution(self):
        solution_dir = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
        self.des_curve_filename = solution_dir+'/Curve_in_base_frame.csv'
        self.des_curvejs_filename = solution_dir+'/Curve_js.csv'
        self.part_pose=np.loadtxt(solution_dir+'/curve_pose.csv',delimiter=',')
        curve_stringidx1=solution_dir.index('data/')
        part_name=solution_dir[curve_stringidx1+5:]
        curve_stringidx2=part_name.index('/')
        part_name=part_name[:curve_stringidx2]
        ###TODO: warning if not a valid solution directory
        part_pose_m=copy.deepcopy(self.part_pose)
        part_pose_m[:-1,-1]=part_pose_m[:-1,-1]/1000.

        self.tes_env.update_pose(part_name,part_pose_m)
        
    def motionProgUpdateRight(self):
        
        self.moProgUpRightBox=QGroupBox('Motion Program Update')

        # add button
        solution_button=QPushButton('Open Solution Directory')
        solution_button.setDefault(True)
        solution_button.clicked.connect(self.read_Solution)
        self.solutionDirtext=QLabel('(Please select solution directory)')

        # descurvebutton=QPushButton('Open Desired Curve File')
        # descurvebutton.setDefault(True)
        # descurvebutton.clicked.connect(self.readDesiredCurveFile)
        # self.des_curve_filenametext=QLabel('(Please add desired curve file)')

        # descurvejsbutton=QPushButton('Open Desired Curve js File')
        # descurvejsbutton.setDefault(True)
        # descurvejsbutton.clicked.connect(self.readDesiredCurveJsFile)
        # self.des_curvejs_filenametext=QLabel('(Please add desired curve js file)')

        filebutton=QPushButton('Open Command File')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCmdFile)
        self.cmd_filenametext=QLabel('(Please add motion command file)')

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


        self.visual_runButton=QPushButton("Run Visualization")
        self.visual_runButton.setDefault(True)
        self.visual_runButton.clicked.connect(self.run_Visualization)
        self.run3_result=QLabel('')


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

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(solution_button)
        layout.addWidget(self.solutionDirtext)

        # layout.addWidget(descurvebutton)
        # layout.addWidget(self.des_curve_filenametext)
        # layout.addWidget(descurvejsbutton)
        # layout.addWidget(self.des_curvejs_filenametext)
        layout.addWidget(filebutton)
        layout.addWidget(self.cmd_filenametext)
        layout.addLayout(vellayout)
        layout.addLayout(tollayout)
        layout.addWidget(self.visual_runButton)
        layout.addWidget(self.moupdate_runButton)
        layout.addWidget(self.run4_result)
        layout.addWidget(self.run4_result_img)
        layout.addStretch(1)
        self.moProgUpRightBox.setLayout(layout)
    
    def readDesiredCurveFile(self):
        
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        dlg.setFilter(QDir.Files)

        if dlg.exec_():
            filenames = dlg.selectedFiles()
            self.des_curve_filename = filenames[0]
            self.des_curve_pathname = os.path.dirname(self.des_curve_filename)
            self.des_curve_filenametext.setText(self.des_curve_filename)
    
    def readDesiredCurveJsFile(self):
        
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        dlg.setFilter(QDir.Files)

        if dlg.exec_():
            filenames = dlg.selectedFiles()
            self.des_curvejs_filename = filenames[0]
            self.des_curvejs_pathname = os.path.dirname(self.des_curvejs_filename)
            self.des_curvejs_filenametext.setText(self.des_curvejs_filename)
    
    def readCmdFile(self):
        
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        dlg.setFilter(QDir.Files)

        if dlg.exec_():
            filenames = dlg.selectedFiles()
            self.cmd_filename = filenames[0]
            self.cmd_pathname = os.path.dirname(self.cmd_filename)
            self.cmd_filenametext.setText(self.cmd_filename)
    
    def run_Visualization(self):
        if self.robot1 is None:
            self.run3_result.setText("Robot1 not yet choosed.")
            return
        
        vel=int(self.vel_box.value())
        errtol=float(self.error_box.value())
        angerrtol=float(self.ang_error_box.value())
        velstdtol=float(self.vel_std_box.value())
        self.run3_result.setText('Running Visualization')
        # qthread for motion update
        self.visual_thread=QThread()
        self.visual_timer_thread=QThread()
        self.visual_timer=Timer()

        curve_js=np.loadtxt(self.des_curvejs_filename,delimiter=',')
        try:    ###TODO: add error box popup
            self.visual_worker=Worker(self.tes_env.viewer_trajectory,self.robot1_name,curve_js[::100])
            self.visual_worker,self.visual_thread,self.visual_timer,self.visual_timer_thread=\
                setup_worker_timer(self.visual_worker,self.visual_thread,self.visual_timer,self.visual_timer_thread,self.prog_Visualization,self.res_Visualization)
        except:
            traceback.print_exc()
            self.showdialog(traceback.format_exc())

        ## edit interface
        self.visual_runButton.setEnabled(False)

        ## final result setup
        self.visual_thread.finished.connect(lambda: self.visual_runButton.setEnabled(True))

        ## start thread
        self.visual_timer_thread.start()
        self.visual_thread.start()



    def run_MotionProgUpdate(self):

        if self.robot1 is None:
            self.run4_result.setText("Robot1 not yet choosed.")
            return

        if self.cmd_filename is None:
            self.run4_result.setText("Command file not yet choosed.")
            return
        
        vel=int(self.vel_box.value())
        errtol=float(self.error_box.value())
        angerrtol=float(self.ang_error_box.value())
        velstdtol=float(self.vel_std_box.value())
        self.run4_result.setText('Updating Motion Program')

        ## delete previous tmp result
        all_files=os.listdir(self.cmd_pathname)
        if self.cmd_pathname+'/result_speed_'+str(vel)+'/' in all_files:
            output_dir=self.cmd_pathname+'/result_speed_'+str(vel)+'/'
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
            self.moupdate_worker=Worker(motion_program_update,self.cmd_pathname,self.robot1,self.robot1MotionSend,vel,self.des_curve_filename,self.des_curvejs_filename,\
                errtol,angerrtol,velstdtol)
            self.moupdate_worker,self.moupdate_thread,self.moupdate_timer,self.moupdate_timer_thread=\
                setup_worker_timer(self.moupdate_worker,self.moupdate_thread,self.moupdate_timer,self.moupdate_timer_thread,\
                    self.prog_MotionProgUpdate,self.res_MotionProgUpdate)
        except:
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
            result_text+='\nCurrent Max Error:'+str(round(np.max(error),2))+' mm, Max Ang Error:'+str(round(np.max(angle_error),2))+' deg'+\
                '\nAve. Speed:'+str(round(np.mean(speed),2))+' mm/sec, Std. Speed:'+str(round(np.std(speed),2))+' mm/sec, Std/Ave Speed:'+str(round(100*np.std(speed)/np.mean(speed),2))+' %'
        self.run4_result.setText(result_text)

    def res_MotionProgUpdate(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run4_result.setText('Motion Program Update. Failed. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        curve_exe_js,speed,error,angle_error,breakpoints,primitives,q_bp,p_bp,run_duration=result
        vel=int(self.vel_box.value())

        # save motion program commands
        # df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
        df=DataFrame({'primitives':primitives,'p_bp':p_bp,'q_bp':q_bp})
        df.to_csv(self.cmd_pathname+'/result_speed_'+str(vel)+'/final_command.csv',header=True,index=False)
        DataFrame(curve_exe_js).to_csv(self.cmd_pathname+'/result_speed_'+str(vel)+'/curve_js_exe.csv',header=False,index=False)

        result_text='Motion Program Update. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration))+\
            '\nCommand file saved at\n'+self.cmd_pathname+'/result_speed_'+str(vel)
        result_text+='\nMax Error:'+str(round(np.max(error),2))+' mm, Max Ang Error:'+str(round(np.max(angle_error),2))+' deg'+\
            '\nAve. Speed:'+str(round(np.mean(speed),2))+' mm/sec, Std. Speed:'+str(round(np.std(speed),2))+' mm/sec, Std/Ave Speed:'+str(round(100*np.std(speed)/np.mean(speed),2))+' %'
        self.run4_result.setText(result_text)



if __name__ == '__main__':

    app=QApplication(sys.argv)

    window=SprayGUI()
    window.show()
    sys.exit(app.exec())
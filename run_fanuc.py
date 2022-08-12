from PyQt5.QtCore import QDateTime,Qt,QTimer,QDir,QObject,QRunnable,pyqtSignal,pyqtSlot,QThread
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QDoubleSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget,QFileDialog)
from PyQt5.QtGui import QPixmap
import sys
from pathlib import Path
from pandas import *
import os
import time

from toolbox.robots_def import *
from functions_fanuc import *

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
        except Exception as e:
            print("There is Error")
            print(e)
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

        # robot box
        robot_list=['','m710ic','m900ia']
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

        ## main layout
        mainLayout = QGridLayout()
        mainLayout.addLayout(toplayout,0,0,1,2)
        mainLayout.addWidget(self.redResLeftBox,1,0)
        mainLayout.addWidget(self.moProgGenMidBox,1,1)
        mainLayout.addWidget(self.moProgUpRightBox,1,2)
        self.setLayout(mainLayout)

        self.setWindowTitle('Spray FANUC GUI')
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
        elif robot1_choose == 'm710ic':
            self.robot1=m710ic(d=50)
        elif robot1_choose == 'm900ia':
            self.robot1=m900ia(d=50)
        self.robot1_name=robot1_choose
        
    def robot2_change(self,robot2_choose):
        print("Robot2 not supported now.")
    
    def redundancyResLeft(self):

        # Group Box
        self.redResLeftBox=QGroupBox("Redundancy Resolution")
        
        # add widgets
        filebutton=QPushButton('Open Curve File')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCurveFile)
        self.curve_filenametext=QLabel('(Please add curve file)')
        self.redres_runButton=QPushButton("Run")
        self.redres_runButton.setDefault(True)
        self.redres_runButton.clicked.connect(self.run_RedundancyResolution)
        self.run1_result=QLabel('')

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(filebutton)
        layout.addWidget(self.curve_filenametext)
        layout.addWidget(self.redres_runButton)
        layout.addWidget(self.run1_result)
        layout.addStretch(1)
        self.redResLeftBox.setLayout(layout)

    def readCurveFile(self):
        
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        dlg.setFilter(QDir.Files)

        if dlg.exec_():
            filenames = dlg.selectedFiles()
            self.curve_filename = filenames[0]
            self.curve_pathname = os.path.dirname(self.curve_filename)
            self.curve_filenametext.setText(self.curve_filename)
    
    def run_RedundancyResolution(self):

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
        self.redres_worker=Worker(redundanct_resolution,self.curve_filename,self.robot1)
        self.redres_timer=Timer()
        

        self.redres_worker,self.redres_thread,self.redres_timer,self.redres_timer_thread=\
            setup_worker_timer(self.redres_worker,self.redres_thread,self.redres_timer,self.redres_timer_thread,\
                self.prog_RedundancyResolution,self.res_RedundancyResolution)

        ## edit interface
        self.redres_runButton.setEnabled(False)
        
        ## final result setup
        self.redres_thread.finished.connect(lambda: self.redres_runButton.setEnabled(True))

        ## start thread
        self.redres_thread.start()
        self.redres_timer_thread.start()
    
    def prog_RedundancyResolution(self,n):
        
        self.run1_result.setText('Solving Redundancy Resolution. Time: '+time.strftime("%H:%M:%S", time.gmtime(n)))
    
    def res_RedundancyResolution(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run1_result.setText('Redundancy Resolution failed because of errors. Time:',time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        curve_base,curve_normal_base,curve_js,H,run_duration=result

        save_filepath=self.curve_pathname+'/'+self.robot1_name+'_fanuc/'
        Path(save_filepath).mkdir(exist_ok=True)

        ###save file
        df=DataFrame({'x':curve_base[:,0],'y':curve_base[:,1], 'z':curve_base[:,2],'x_dir':curve_normal_base[:,0],'y_dir':curve_normal_base[:,1], 'z_dir':curve_normal_base[:,2]})
        df.to_csv(save_filepath+'Curve_in_base_frame.csv',header=False,index=False)
        DataFrame(curve_js).to_csv(save_filepath+'Curve_js.csv',header=False,index=False)
        with open(save_filepath+'blade_pose.yaml', 'w') as file:
            documents = yaml.dump({'H':H.tolist()}, file)

        self.run1_result.setText('Redundancy Resolution Solved\nFile Path:\n'+save_filepath+'\nTotal Time: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))

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
        self.moprog_runButton=QPushButton("Run")
        self.moprog_runButton.setDefault(True)
        self.moprog_runButton.clicked.connect(self.run_MotionProgGeneration)
        self.run2_result=QLabel('')

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(filebutton)
        layout.addWidget(self.curvejs_filenametext)
        layout.addWidget(seglabel)
        layout.addWidget(self.total_seg_box)
        layout.addWidget(self.moprog_runButton)
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
    
    def run_MotionProgGeneration(self):

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
        self.moprog_worker=Worker(motion_program_generation,self.curvejs_filename,self.robot1,total_seg)

        self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread=\
            setup_worker_timer(self.moprog_worker,self.moprog_thread,self.moprog_timer,self.moprog_timer_thread,\
                self.prog_MotionProgGeneration,self.res_MotionProgGeneration)

        ## edit interface
        self.moprog_runButton.setEnabled(False)
        self.total_seg_box.setEnabled(False)
        ## final result setup
        self.moprog_thread.finished.connect(lambda: self.moprog_runButton.setEnabled(True))
        self.moprog_thread.finished.connect(lambda: self.total_seg_box.setEnabled(True))

        ## start thread
        self.moprog_timer_thread.start()
        self.moprog_thread.start()
    
    def prog_MotionProgGeneration(self,n):
        
        self.run2_result.setText('Generating Motion Program. Time: '+time.strftime("%H:%M:%S", time.gmtime(n)))

    def res_MotionProgGeneration(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run2_result.setText('Motion Program Generation Failed. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        breakpoints,primitives,q_bp,p_bp,run_duration=result

        # save motion program commands
        df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'points':p_bp,'q_bp':q_bp})
        df.to_csv(self.curvejs_pathname+'/command.csv',header=True,index=False)

        self.run2_result.setText('Motion Program Generated\nTime: '+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
        
    def motionProgUpdateRight(self):
        
        self.moProgUpRightBox=QGroupBox('Motion Program Update')

        # add button
        descurvebutton=QPushButton('Open Desired Curve File')
        descurvebutton.setDefault(True)
        descurvebutton.clicked.connect(self.readDesiredCurveFile)
        self.des_curve_filenametext=QLabel('(Please add desired curve file)')

        descurvejsbutton=QPushButton('Open Desired Curve js File')
        descurvejsbutton.setDefault(True)
        descurvejsbutton.clicked.connect(self.readDesiredCurveJsFile)
        self.des_curvejs_filenametext=QLabel('(Please add desired curve js file)')

        filebutton=QPushButton('Open Command File')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCmdFile)
        self.cmd_filenametext=QLabel('(Please add motion command file)')

        # box for vel
        self.vel_box = QSpinBox()
        self.vel_box.setMinimum(1)
        self.vel_box.setMaximum(2000)
        self.vel_box.setValue(700)
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

        self.moupdate_runButton=QPushButton("Run")
        self.moupdate_runButton.setDefault(True)
        self.moupdate_runButton.clicked.connect(self.run_MotionProgUpdate)
        self.run3_result=QLabel('')
        self.run3_result_img=QLabel('')

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
        layout.addWidget(descurvebutton)
        layout.addWidget(self.des_curve_filenametext)
        layout.addWidget(descurvejsbutton)
        layout.addWidget(self.des_curvejs_filenametext)
        layout.addWidget(filebutton)
        layout.addWidget(self.cmd_filenametext)
        layout.addLayout(vellayout)
        layout.addLayout(tollayout)
        layout.addWidget(self.moupdate_runButton)
        layout.addWidget(self.run3_result)
        layout.addWidget(self.run3_result_img)
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
    
    def run_MotionProgUpdate(self):

        if self.robot1 is None:
            self.run3_result.setText("Robot1 not yet choosed.")
            return
        if self.cmd_filenametext is None:
            self.run3_result.setText("Command file not yet choosed.")
            return
        
        vel=int(self.vel_box.value())
        errtol=float(self.error_box.value())
        angerrtol=float(self.ang_error_box.value())
        velstdtol=float(self.vel_std_box.value())
        self.run3_result.setText('Updating Motion Program')

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
        
        # qthread for redundancy resolution
        self.moupdate_thread=QThread()
        self.moupdate_timer_thread=QThread()
        self.moupdate_timer=Timer()
        self.moupdate_worker=Worker(motion_program_update,self.cmd_pathname,self.robot1,vel,self.des_curve_filename,self.des_curvejs_filename,\
            errtol,angerrtol,velstdtol)

        self.moupdate_worker,self.moupdate_thread,self.moupdate_timer,self.moupdate_timer_thread=\
            setup_worker_timer(self.moupdate_worker,self.moupdate_thread,self.moupdate_timer,self.moupdate_timer_thread,\
                self.prog_MotionProgUpdate,self.res_MotionProgUpdate)

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
        all_files=os.listdir(output_dir)

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
            self.run3_result_img.setPixmap(pixmap)
        
        result_text='Updating Motion Program. Time:'+time.strftime("%H:%M:%S", time.gmtime(n))
        if speed is not None and error is not None and angle_error is not None:
            result_text+='\nCurrent Max Error:'+str(round(np.max(error),2))+' mm, Max Ang Error:'+str(round(np.max(angle_error),2))+' deg'+\
                '\nAve. Speed:'+str(round(np.mean(speed),2))+' mm/sec, Std. Speed:'+str(round(np.std(speed),2))+' mm/sec, Std/Ave Speed:'+str(round(100*np.std(speed)/np.mean(speed),2))+' %'
        self.run3_result.setText(result_text)

    def res_MotionProgUpdate(self,result):

        if len(result) <= 1:
            run_duration=result[0]
            self.run3_result.setText('Motion Program Update. Failed. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration)))
            return

        curve_exe_js,speed,error,angle_error,breakpoints,primitives,q_bp,p_bp,run_duration=result
        vel=int(self.vel_box.value())

        # save motion program commands
        # df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'points':p_bp,'q_bp':q_bp})
        df=DataFrame({'primitives':primitives,'points':p_bp,'q_bp':q_bp})
        df.to_csv(self.cmd_pathname+'/result_speed_'+str(vel)+'/final_command.csv',header=True,index=False)
        DataFrame(curve_exe_js).to_csv(self.cmd_pathname+'/result_speed_'+str(vel)+'/curve_js_exe.csv',header=False,index=False)

        result_text='Motion Program Update. Time:'+time.strftime("%H:%M:%S", time.gmtime(run_duration))+\
            '\nCommand file saved at\n'+self.cmd_pathname+'/result_speed_'+str(vel)
        result_text+='\nMax Error:'+str(round(np.max(error),2))+' mm, Max Ang Error:'+str(round(np.max(angle_error),2))+' deg'+\
            '\nAve. Speed:'+str(round(np.mean(speed),2))+' mm/sec, Std. Speed:'+str(round(np.std(speed),2))+' mm/sec, Std/Ave Speed:'+str(round(100*np.std(speed)/np.mean(speed),2))+' %'
        self.run3_result.setText(result_text)



if __name__ == '__main__':

    app=QApplication(sys.argv)

    window=SprayGUI()
    window.show()
    sys.exit(app.exec())
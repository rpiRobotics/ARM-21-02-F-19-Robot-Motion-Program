from PyQt5.QtCore import QDateTime,Qt,QTimer,QDir,QObject,QRunnable,pyqtSignal,pyqtSlot
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QDoubleSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget,QFileDialog)
import sys
from pathlib import Path
from pandas import *
import os
import time

from toolbox.robots_def import *
from functions_fanuc import *

class WorkerSignals(QObject):
    '''
    Defines the signals available from a running worker thread.

    Supported signals are:

    finished
        No data

    error
        tuple (exctype, value, traceback.format_exc() )

    result
        object data returned from processing, anything

    progress
        int indicating % progress

    '''
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    result = pyqtSignal(object)
    progress = pyqtSignal(int)

class Worker(QRunnable):
    '''
    Worker thread

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''
    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()

        # Add the callback to our kwargs
        self.kwargs['progress_callback'] = self.signals.progress

    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''

        # Retrieve args/kwargs here; and fire processing using them
        try:
            result = self.fn(*self.args, **self.kwargs)
        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))
        else:
            self.signals.result.emit(result)  # Return the result of the processing
        finally:
            self.signals.finished.emit()  # Done

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
        runButton=QPushButton("Run")
        runButton.setDefault(True)
        runButton.clicked.connect(self.run_RedundancyResolution)
        self.run1_result=QLabel('')

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(filebutton)
        layout.addWidget(self.curve_filenametext)
        layout.addWidget(runButton)
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
        try:
            curve_base,curve_normal_base,curve_js,H=redundanct_resolution(self.curve_filename,self.robot1)
        except:
            print("There is Error")
        
        save_filepath=self.curve_pathname+'/'+self.robot1_name+'_fanuc/'
        Path(save_filepath).mkdir(exist_ok=True)

        ###save file
        df=DataFrame({'x':curve_base[:,0],'y':curve_base[:,1], 'z':curve_base[:,2],'x_dir':curve_normal_base[:,0],'y_dir':curve_normal_base[:,1], 'z_dir':curve_normal_base[:,2]})
        df.to_csv(save_filepath+'Curve_in_base_frame.csv',header=False,index=False)
        DataFrame(curve_js).to_csv(save_filepath+'Curve_js.csv',header=False,index=False)
        with open(save_filepath+'blade_pose.yaml', 'w') as file:
            documents = yaml.dump({'H':H.tolist()}, file)

        self.run1_result.setText('Redundancy Resolution Solved\nFile Path:\n'+save_filepath)

    def motionProgGenMid(self):

        # Group Box
        self.moProgGenMidBox=QGroupBox('Motion Program Generation')

        # add button
        filebutton=QPushButton('Open Curve File')
        filebutton.setDefault(True)
        filebutton.clicked.connect(self.readCurveJsFile)
        self.curvejs_filenametext=QLabel('(Please add curve joint space file)')
        self.total_seg_box = QSpinBox(self.moProgGenMidBox)
        self.total_seg_box.setMinimum(1)
        self.total_seg_box.setMaximum(50000)
        self.total_seg_box.setValue(100)
        seglabel=QLabel('Total Segments:')
        seglabel.setBuddy(self.total_seg_box)
        runButton=QPushButton("Run")
        runButton.setDefault(True)
        runButton.clicked.connect(self.run_MotionProgGeneration)
        self.run2_result=QLabel('')

        # add layout
        layout = QVBoxLayout()
        layout.addWidget(filebutton)
        layout.addWidget(self.curvejs_filenametext)
        layout.addWidget(seglabel)
        layout.addWidget(self.total_seg_box)
        layout.addWidget(runButton)
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

        total_seg=int(self.total_seg_box.value())
        print(total_seg)
        self.run2_result.setText('Generating Motion Program')
        try:
            breakpoints,primitives,q_bp,p_bp = motion_program_generation(self.curvejs_filename,self.robot1,total_seg)
        except:
            print("There is error.")

        # save motion program commands
        df=DataFrame({'breakpoints':breakpoints,'primitives':primitives,'points':p_bp,'q_bp':q_bp})
        df.to_csv(self.curvejs_pathname+'/command.csv',header=True,index=False)

        self.run2_result.setText('Motion Program Generated')

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

        runButton=QPushButton("Run")
        runButton.setDefault(True)
        runButton.clicked.connect(self.run_MotionProgUpdate)
        self.run3_result=QLabel('')

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
        layout.addWidget(runButton)
        layout.addWidget(self.run3_result)
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
        
        motion_program_update(self.cmd_pathname,self.robot1,vel,self.des_curve_filename,self.des_curvejs_filename,\
            errtol,angerrtol,velstdtol)

        self.run3_result.setText('Motion Program Update.\nCommand file saved at\n'+self.cmd_pathname+'/result_speed_'+str(vel))

if __name__ == '__main__':

    app=QApplication(sys.argv)

    window=SprayGUI()
    window.show()
    sys.exit(app.exec())
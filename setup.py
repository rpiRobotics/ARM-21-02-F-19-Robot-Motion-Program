from setuptools import setup
###SET GAZEBO_MODEL_PATH
#tesseract, tesseract-viewer
setup(
    name='motion_primitive',
    version='0.0.1',
    description='ARM-21-02-F-19-Robot-Motion-Program',
    url='https://github.com/rpiRobotics/ARM-21-02-F-19-Robot-Motion-Program',
    py_modules=[],
    install_requires=[
        'PyQt5',
        'numpy',
        'pandas',
        'general_robotics_toolbox',
        'matplotlib',
        'scipy',
        'qpsolvers',
        'sklearn',
        'tesseract-robotics',
        'tesseract-robotics-viewer'
    ]
)
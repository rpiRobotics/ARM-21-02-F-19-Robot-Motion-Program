from setuptools import setup
###SET GAZEBO_MODEL_PATH
#tesseract, tesseract-viewer
setup(
    name='motion_primitive',
    version='0.0.1',
    description='Motion Primitive Project Tools',
    url='https://github.com/hehonglu123/Motion-Primitive-Planning',
    py_modules=[],
    install_requires=[
        'PyQt5',
        'numpy',
        'pandas',
        'general_robotics_toolbox',
        'matplotlib',
        'sklearn'
    ]
)
# ARM-21-02-F-19-ROBOT-MOTION-PROGRAM

The repository for ARM project ARM-21-02-F-19-ROBOT-MOTION-PROGRAM

## GUI

The GUI include the whole process of generating and optimizing the motion program, including.
- Solving Redundancy Resolution (left). Determining the curve pose and the joint space trajectory.
- Motion Program Generation (mid). From the curve pose and the joint space trajectory, generate a robot motion program.
- Motion Program Update (right). Update motion program to further optimized it to satisfied requirements.

Please click [here](https://github.com/rpiRobotics/ARM-21-02-F-19-Robot-Motion-Program/tree/main/doc/gui_manual.md) for detail instruction.

![](doc/figures/gui.png)

Please note that the GUI is an ongoing developed feature. Contact the authors for further questions or requests.

The GUI currently only support FANUC robot.

## Baseline

TBA

## Max Gradient and Iterative Learning Control (ILC)

FANUC Multi-Peak Max Gradient Descent [Link](https://github.com/rpiRobotics/ARM-21-02-F-19-Robot-Motion-Program/tree/main/ILC)
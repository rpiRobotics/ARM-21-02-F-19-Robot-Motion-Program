# Max Gradient and Iterative Learning Control (ILC)

## FANUC

#### Pre-request Libraries

- python3 (The code were tested with python3.9)
- numpy
- scipy
- pandas
- rpi_general_robotics_toolbox_py [link](https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py) Download the repo and do `pip install . --user`
- fanuc_motion_program_exec [link](https://github.com/eric565648/fanuc_motion_program_exec). The module provides interface to generate motion programs (i.e. LS for FANUC) using python.


### (Multi-Peak) Max Gradient Descent

In each iteration, the program first finds multiple peaks in the euclidean error. Then it uses blending model to generate gradient direction to minimize error through adjusting breakpoints. Finally, the error from the robot is used with the gradient direction to minimize the error.

<p align="center">
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_0.png" width="180" />
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_1.png" width="180" /> 
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_2.png" width="180" />
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_3.png" width="180" />
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_4.png" width="180" /> 
</p>
<p align="center">
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_5.png" width="180" />
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_6.png" width="180" /> 
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_7.png" width="180" />
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_8.png" width="180" />
  <img src="max_gradient/fanuc/curve_1_speed_300/iteration_9.png" width="180" /> 
</p>

#### Execution

1. Open Visual Studio Code and open terminal with `ctrl j`
2. In the terminal, go to directory `ILC`
```
cd D:\ARM-21-02-F-19-Robot-Motion-Program\ILC
```
3. Run `MAIN.LS` in RoboGuide as showed in [fanuc_motion_program_exec](https://github.com/eric565648/fanuc_motion_program_exec) (Choose `MAIN` and press the play button)
4. Use python with arguments to execute max gradient descent on `curve_1` or `curve_2` with different command speed.
```
python max_gradient_fanuc.py --curve curve_1 --speed 300
```
5. In the terminal, you will see output like below
```
Multi-Peak Max Gradient. Curve: curve_1 , Command Speed: 300
Total Iterations: 10
Iteration: 1 , Max Error: 0.9314343900690465 , Ave. Speed: 213.32511694157316
Iteration: 2 , Max Error: 0.8515334478789969 , Ave. Speed: 213.35040542197083
Iteration: 3 , Max Error: 0.7663115295622082 , Ave. Speed: 213.3644769981829
Iteration: 4 , Max Error: 0.673624536302329 , Ave. Speed: 213.37617700570928
Iteration: 5 , Max Error: 0.625052814174626 , Ave. Speed: 213.38860869178285
Iteration: 6 , Max Error: 0.5503149151531893 , Ave. Speed: 213.39959475102472
...
```
6. In the folder `(repository directory)\ILC\max_gradient\fanuc\(curve)_speed_(speed)`, you will see error/speed plots of each interation.
7. You can further adjust the total interation by adding `--iteration` arguments. The default total iteration is `10`.
```
python max_gradient_fanuc.py --curve curve_1 --speed 300 --iteration 20
```
8. Note that the execution result will be overwritten with the same curve and the same speed.

#### Result Interpretation
<p align="center">
<img src="max_gradient/fanuc/curve_1_speed_300/iteration_3.png" width="500"/>
</p>

The green curve is the speed of the spray gun relative to the curve. The scale is showed on the left with the unit `mm/sec`. The blue curve is the euclidean distance (position error) to the curve and the yellow curve is the normal angular error. The scale is showed on the right with the unit `mm` and `deg` respectively. 

The blue dot(s) on the blue curve are the error peak to be minimized.

# GUI Manual

The whole process process including three steps, Solving Redundancy Resolution, Motion Program Generation, Motion Program Update. Each step may have multiple algorithms/methods. The feature of allowing user to choose from methods will be added in the future.

![](figures/gui.png)

See [here](https://youtu.be/qCv11wtNU88) for video instruction.

## Pre-request

A csv file defined the desired curve path (position and normal direction) is requered. The first three columns are position (xyz) and the last three columns are normal direction (xyz). An example file is as followed.

```
...
1.9316380763831253,0.001054979200265463,25.39999998904546,-0.0,-2.076730708736155e-05,-0.9999999997843596
1.952187417621244,0.001077544961782589,25.39999998857182,-0.0,-2.121151499094843e-05,-0.9999999997750358
1.9727367588593623,0.001100349514426261,25.39999998808298,-0.0,-2.1660423507246714e-05,-0.999999999765413
1.9932861000974806,0.0011233928581964457,25.39999998757862,-0.0,-2.21140326362551e-05,-0.9999999997554847
2.0138354413355986,0.001146674993093109,25.399999987058422,-0.0,-2.2572342377972192e-05,-0.9999999997452446
...
```

## Run Gui

Execute `start.py` in the directory where you saved the repo.

```
cd (repo directory)
python start.py
```

## Solving Redundancy Resolution

The step finds the optimal curve pose (relative to the robot base frame), the curve represented in base frame and the curve in joint space (joints trajectories).

### Baseline

0. Choose Robot1
1. Click `Open Curve File` and choose the prepared curve file described in pre-request.
2. Click `Run`.
3. The result will be saved in a folder named `(robot1)_fanuc` in the same directory of the curve file. 

![](figures/redres_folder.png)

4. The result includes the curve pose (`blade_pose.yaml`), the curve represented in base frame (`Curve_in_base_frame.csv`) and the curve in joint space (`Curve_js.csv`)

![](figures/redres_result.png)

## Motion Program Generation

The step generate a motion program given the curve in joint space file.

### Baseline

The baseline method simply divided the trajectory into equal distance `Move L` (linear move in  cartesian space).

0. Choose Robot1
1. Click `Open Curve Js File` and open the previous generated `Curve_js.csv`.
2. Enter how many moveL you want.
3. Click `Run`
4. The result will be saved in a directory of `Curve_js.csv` named `command.csv`.

![](figures/moproggen_baseline_result.png)

## Motion Program Update

### Multi Max Gradient Descent

0. Choose Robot1
1. Click `Open Desired Curve File` and open the previous generated `Curve_in_base_frame.csv`.
2. Click `Open Desired Curve js File` and open the previous generated `Curve_js.csv`.
3. Click `Open Command File` and open the previous generated `command.csv`.
4. Enter the desired velocity, euclidean error/angular error/speed variantion tolerance.
5. **!!! Important !!!** For FANUC: Make sure the robot in the roboguide is running (The play key is clicked and turnes green, running the main program. See [here](https://github.com/eric565648/fanuc_motion_program_exec) for further information.)
6. Click `Run`
7. While running the program, the result of each iteration will be show in the panel. The result will be saved in a folder `result_speed_(vel)` in the same directory of the motion command profile `command.csv`.

![](figures/max_grad_folder.png)

8. The result includes speed/error plots and updated commands through iterations (`iteration_(x).png`,`command_(x).png`) and the final euclidean error/angular error/speed, speed/error plots commands (`final_error.npy`,`final_ang_error.npy`,`final_speed.npy`,`final_command.csv`,`final_iteration.png`). The final files are the same as the last iteration files.

![](figures/max_grad_result.png)


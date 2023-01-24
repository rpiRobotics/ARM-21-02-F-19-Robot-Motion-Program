# Robot Configuration

To add a new robot to the system, two things are required.
1. Add robot name to `robot_lists.yml` file.
2. A robot configuration file.

## Add robot name to `robot_lists.yml` file

Please add the new robot name to the new line with a desh and a blank as the beginning. The robot must has name `(manufacturer)_(model)`, i.e.

```
- (manufacturer)_(model)
```
For example,
```
- FANUC_lrmate200id
- ABB_1200_5_90
```

## Add robot configuration file.

The following command can convert `robot info file` to `robot configuration file`.
```
python robot_info_to_config.py
```

An example `robot info file` is like `lrmate200id_info.yml`. You can copy paste a new file and change the parameters inside the new file. Below are `robot info file` paramters explaination.

### Robot info file explaination

- device name: The device name of the robot. This parameter does not really affect anything.
- manufacturer: The manufacturer of the robot.
- model: The model name of the robot.
- user_description: User description. This can be anything
- axis: The rotation axis when the robot is at zero configuration (i.e. when all the joint is at `0` degree), with x-axis pointing forward, y-axis pointing left and z-axis pointing upward. 
- chain: The link vector from joint frame origin to origin. There are tons of ways to define the origin of joint frame as long as they are on the joint rotation axis. The numbers from robot manual can definitely help you define the origins, and is probably required. An example way to define the link is here
    - First link vector: From center of the base, going in z-axis, to the same height as joint 2.
    - Second link vector: From the previous point to the center of joint 2.
    - Third link vector: From the center of joint 2 to the center of joint 3.
    - Forth link vector: From the center of joint 3 to the center of joint 5.
    - Fifth link vector: all zero
    - Sixth link vector: all zero
    - Seventh link vector: From the center of joint 5 to the center of the flange.
- joint_limits_lower: Joint rotation lower limits. Please refer to the robot manual for the actual number.
- joint_limits_upper: Joint rotation upper limits. Please refer to the robot manual for the actual number.
- joint_vel_limits: Joint velocity limits. Please refer to the robot manual for the actual number.

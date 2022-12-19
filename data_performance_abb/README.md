# Data Format for Motion Primitive Planning

## Original Spatial Curve Data
This repo contains two sets of spatial curve, represented in its own Cartesian frame with first three elements of xyz point and last three elements of xyz normal vector.
Those files are either created with analytical expression: `data/wood/Curve_dense.csv` or extracted from NX: `data/from_NX/Curve_dense.csv`.

## Spatial Curve Pose
This is usually a csv file with a homogeneous transformation matrix, representing the curve frame with respect to robot base frame: `data/wood/curve_pose_opt1/curve.csv`

## Spatial Curve in Robot Base Frame
With known curve pose, the spatial curve is converted into robot base frame, with same point + vector format:  `data/wood/curve_pose_opt1/Curve_in_base_frame.csv`

## Spatial Curve in Joint Space Representation
This is robot joint configuration file for all points in `Curve_in_base_frame.csv`, solved with redundancy resolution. Each row is a point in 6d joint space: `data/wood/curve_pose_opt1/Curve_js.csv`

## Robot Motion Command File
This file contains a series of breakpoints information: `data/wood/curve_pose_opt1/100L/command.csv`, including 
* breakpoint index in `Curve_in_base_frame.csv`
* segment primitive type
* breakpoint(s) in robot base frame. If the primitive type is MoveJ or MoveL, the length of this argument is 1. If the primitive type is MoveC, the length of this argument is 2.
* breakpoint(s) in joint space. Similarly, 2 for MoveC and 1 for MoveJ and MoveL

## Interpolated Trajectory from Motion Command
This file contains a continuous (not smooth) trajectory interpolated from motion primitives: `curve_fit.csv`, and in joint space `curve_fit_js.csv`

## Dual Arm Relative Pose
`data/wood/dual_arm/diffevo1/base.csv` contains the homogeneous transformation matrix of 1200 base frame relative to 6640's base frame.

## Spatial Curve Pose held by Tool
`data/wood/dual_arm/diffevo1/tcp.csv` contains the homogeneous transformation matrix of spatial curve frame relative to 1200 end-effector frame.

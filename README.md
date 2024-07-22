# landing_controller
Require locosim (https://github.com/mfocchi/locosim) with

- robot_control branch: traj_optimization
- all robot_descriptions branch: master
- all robot_hardwre_interfaces branch: master
- ros_impedance_controller branch: master


### Hints
Remember that if you want to record video of the simulations, you must:

1. install ffmpeg (https://ffmpeg.org/)
2. initialize Gazebo with camera_slow.world

If you do not want to run the Gazebo client, you can add `'gui:=False'` as additional argument.

`p.startController(world_name='camera_slow.world', additional_args=['gui:=False'])`


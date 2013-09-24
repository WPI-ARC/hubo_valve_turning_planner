hubo_valve_turning_planner
==========================

Python scripts for Comps to plan valve turning tasks and ROS interface to the planner

## Dependencies & Building

The robot models used by the planner are provided by

 * [drchubo](https://github.com/daslrobotics/drchubo) : Models of DRC Hubo for simulation

For planning it relies on
  
 * [comps](http://sourceforge.net/projects/comps) : the Constrained Manipulation Planning Suite, an openrave plugin

For testing with RViz GUI the following packages are needed
    
 * [hubo_ros_core](https://github.com/WPI-ARC/hubo_ros_core) : ros suite that defines messages and provides a ROS interface to hubo
 * [localization_tools](https://github.com/WPI-ARC/localization_tools) : frontend localization tools using interactive markers and backend tools using ICP

All ROS packages should be placed in a catkin workspace simply run the following
    
    catkin_make

## Launching

The GUI testing pipline should be launched with the following commands

    roslaunch hubo_launch display_drchubo_state.launch
    roslaunch valve_planner valve_planner_sim.launch 
    rosrun valve_localization valve_localizer.py
    rosrun rviz rviz


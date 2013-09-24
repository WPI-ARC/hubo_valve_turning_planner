hubo_valve_turning_planner
==========================

Python scripts for Comps to plan valve turning tasks and a ROS interface to the planner

## Building

For the planner it needs
  
 * [comps](http://sourceforge.net/projects/comps) : the Constrained Manipulation Planning Suite, an openrave plugin

For testing with RViz GUI it needs
    
 * [hubo_ros_core](https://github.com/WPI-ARC/hubo_ros_core) : ros suite that defines messages and provides a ROS interface to hubo
 * [localization_tools](https://github.com/WPI-ARC/localization_tools) : frontend localization tools using interactive markers and backend tools using ICP

This stack compilation is catkinized
    
    catkin_make

## Launching

    roslaunch hubo_launch display_drchubo_state.launch
    roslaunch valve_planner valve_planner_sim.launch 
    rosrun valve_localization valve_localizer.py
    rosrun rviz rviz


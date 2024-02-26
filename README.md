

# MuJoCo-ROS


### Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Prequisites](#prequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Join Us](#join-us)
- [Author](#author)
- [Acknowledgements](#acknowledgements)
- [Status](#status)

 ### Introduction 
In today's fast-paced tech world, simulations are like a secret weapon for crafting robots. Our goal? To connect the world of Robotic Operating Systems [(ROS)](https://www.ros.org/) with the awesome physics simulation capabilities of DeepMind's  [MuJoCo](https://mujoco.org/).

### Features
This project integrates the MuJoCo simulation environment with the ros-control package through a hardware interface. By doing this, we are able to simulate and control a robotic manipulator via ROS. mujoco_sim_node, publishes joint states of the robot and takes effort inputs from the hardware interface. 

![rosgraph](https://github.com/METU-KALFA/mujoco_ros/assets/66975290/07859160-d358-45bf-b413-09add683ed9f)

As evident in the node-graph, the project is equipped with RViz for visualization. With our intention to introduce a motion planner and potentially other sensors, RViz will play a pivotal role in visualizing these upcoming additions.

![RVIZ_MUJOCO](https://github.com/METU-KALFA/mujoco_ros/assets/66975290/64702602-5cec-4bcc-b765-404623413a13)

The project currently supports only the UR5e Robotic Manipulator, but our goal is to expand support to other robotic manipulators and possibly even mobile robots in the future.



***To Do:***
- [ ] Adding support for Franka Panda
- [ ] Bringing in MoveIt Support
- [ ] Fine-tuning PID settings

### Prequisites
To get rolling, you'll need:
* ros-noetic
* ros_control
* rospy = 1.15.15
* mujoco-py = 2.3.7
* numpy = 1.24.1 

### Installation
Download the package and build it via catkin. 

### Usage
Launch ROS bridge:

 	roslaunch ur5e_mujoco_bringup ur5_mujoco.laun6ch
  
Launch the simulation:

 	python3 src/mujoco_sim_pkg/scripts/ur5_simulation.py

Launch MoveIt Controllers (Make sure ros_controllers didn't crashed):

 	roslaunch ur5e_moveit_mj ros_controllers.launch

 	roslaunch ur5e_moveit_mj move_group.launch

Now you can control the trajectory of the robot, using MoveIt.


### Join Us!
We want your help! You can copy our code, add to it, report problems, and team up with others who love robots and simulations. Add support to other robots, add different controllers etc. Sky is the limit!

### Author
Yunus Talha Erzurumlu : yunus.terzurumlu@gmail.com

### Acknowledgements
Check these sites:
* https://www.ros.org/
* http://wiki.ros.org/ros_control
* https://mujoco.org/ 

### Status
This project still under the development and mangaed by Yunus Talha Erzurumlu.



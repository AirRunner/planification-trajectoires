#!/bin/bash

## Partie 1.2

# Install packages
ln -s /home/ros/Documents/ROS/nouveau_package_pr2/pr2_moveit_base2 /home/ros/catkin_ws/src

cp /home/ros/Documents/ROS/nouveau_package_pr2/base.urdf.xacro /home/ros/catkin_ws/src/pr2_common/pr2_description/urdf/base_v0

# Install pip
sudo apt install python-pip
pip install statistics

# Launch ROSPlan
rosplan pr2_moveit_base demo.launch

# Launch Python script
python OMPL_algorithm_benchmark_2019.py

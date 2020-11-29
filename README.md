# Rostest
Minimal example of node ROS and its testing

# What does the project do?

In this project we write tests C++ for our package. 
We create C++ Unit Tests, ROS Unit Tests.

# Who is the project for?

This project is for testers in the ROS environment.

# How do I use it:

Install ROS Melodic: http://wiki.ros.org/Installation/Ubuntu

# How to build: 

catkin_make

# How to run:

source devel/setup.bash
start roscore
rosrun rostest rostest_node 

# How to test:

Unit tests:
start roscore
catkin_make run_tests

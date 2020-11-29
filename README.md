# Rostest
Minimal example of node ROS and its testing

# What does the project do?

In this project we write tests C++ for our package. 
We create C++ Unit Tests, ROS Unit Tests.

C++ unit tests: These are the tests without ROS. This means that they are not meant to test any ROS-related issue. These tests are performed using the gtest framework.

ROS unit tests: These will test your ROS code related to a single node. They will start your node and test its external API, like published or subscribed topics, etc. These tests are performed using a combination of a rostest alongside a gtest.

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

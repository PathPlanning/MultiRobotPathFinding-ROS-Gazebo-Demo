#!/bin/bash

# installing ROS
echo Now installing ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

# installing Gazebo

echo Now installing Gazebo
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control


# installikn Turtlebot packages
echo Now installing Turtlebot packages
sudo apt install ros-kinetic-turtlebotsudo apt install ros-kinetic-turtlebot

echo "----------------------------------------------"
echo "----------------------------------------------"
echo ROS Kinetic Turtlebot install finished.
echo "----------------------------------------------"
echo "----------------------------------------------"

exit 0

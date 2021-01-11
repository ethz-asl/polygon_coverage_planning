#!/bin/bash -e
echo "Running the prepare script for polygon_coverage_planning.";
ROS_VERSION=`rosversion -d`
echo "ROS version: ${ROS_VERSION}"

# Build dependencies.
sudo apt install -y python-wstool python-catkin-tools

# Package dependencies.
echo "Installing MAV_COMM dependencies."
sudo apt install -y ros-${ROS_VERSION}-mav-msgs ros-${ROS_VERSION}-mav-planning-msgs
echo "Installing CGAL dependencies."
sudo apt install -y libgmp-dev libmpfr-dev
echo "Installing MONO dependencies."
sudo apt install -y mono-devel
echo "Installig GLOG dependencices."
sudo apt install -y libgoogle-glog

#!/bin/bash -e
echo "Running the prepare script for polygon_coverage_planning.";
ROS_VERSION=`rosversion -d`
echo "ROS version: ${ROS_VERSION}"

# Build dependencies.
sudo apt-get install -y python-wstool python-catkin-tools

# Package dependencies.
echo "Installing MAV_COMM dependencies."
sudo apt-get install -y ros-${ROS_VERSION}-mav-msgs ros-${ROS_VERSION}-mav-planning-msgs
echo "Installing CGAL dependencies."
sudo apt-get install -y libgmp-dev libmpfr-dev
echo "Installing MONO dependencies."
sudo apt-get install -y mono-devel

cd ~/workspace/polygon_coverage_planning/src/install/
csc hello.cs
mono hello.exe

dpkg -l | grep mcs

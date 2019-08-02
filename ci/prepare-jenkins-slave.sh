#!/bin/bash -e
echo "Running the prepare script for polygon_coverage_planning.";
ROS_VERSION=`rosversion -d`
echo "ROS version: ${ROS_VERSION}"

# Build dependencies.
sudo apt install -y python-wstool python-catkin-tools ros-${ROS_VERSION}-cmake-modules

# Package dependencies.
echo "Installing CGAL dependencies."
sudo apt install -y libgmp-dev libmpfr-dev
echo "Installing MONO dependencies."
sudo apt install -y mono-devel

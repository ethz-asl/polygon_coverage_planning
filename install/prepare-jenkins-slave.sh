#!/bin/bash -e
echo "Running the prepare script for polygon_coverage_planning.";
ROS_VERSION=`rosversion -d`
echo "ROS version: ${ROS_VERSION}"

# Build dependencies.
sudo apt-get install -y python3-wstool python3-catkin-tools

# Package dependencies.
echo "Installing CGAL dependencies."
sudo apt-get install -y libcgal-dev
sudo apt-get install -y libgmp-dev libmpfr-dev
echo "Installig GLOG dependencices."
sudo apt-get install -y libgoogle-glog-dev

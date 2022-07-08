#!/bin/bash -e
echo "Running the prepare script for polygon_coverage_planning.";
ROS_VERSION=`rosversion -d`
echo "ROS version: ${ROS_VERSION}"

# Build dependencies.
if [[ $ROS_VERSION = 'melodic' ]]
then
  sudo apt install -y python-wstool python-catkin-tools
else
  sudo apt install -y python3-wstool python3-catkin-tools
fi

# Package dependencies.
echo "Installing CGAL dependencies."
sudo apt install -y libcgal-dev
echo "Installing MONO dependencies."
sudo apt install -y mono-devel
echo "Installig GLOG dependencices."
sudo apt install -y libgoogle-glog-dev

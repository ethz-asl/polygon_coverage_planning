# polygon_coverage_planning
This package contains implementations to compute coverage patterns and shortest paths in general polygon with holes.
Please cite our [accompanying publication](https://arxiv.org/pdf/1907.09224) when using it.
```
Bähnemann, Rik, et al.
"Revisiting Boustrophedon Coverage Path Planning as a Generalized Traveling Salesman Problem."
Field and Service Robotics. Springer, Cham, 2019.
```
Video: [https://youtu.be/u1UOqdJoK9s](https://youtu.be/u1UOqdJoK9s)


![Coverage Planning in RVIZ](https://user-images.githubusercontent.com/11293852/61134221-70d18980-a4bf-11e9-87a7-d599b60c8dd2.gif)

## Installation on Ubuntu 18.04 and ROS melodic
Install [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).

Install catkin and wstool build dependencies.
```
sudo apt-get install python-wstool python-catkin-tools ros-melodic-cmake-modules
```

Create a workspace.
```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config --extend /opt/ros/melodic
```

Download package dependencies from [dependencies.rosinstall](install/dependencies.rosinstall).
**Note**: If you have not setup [SSH keys in GitHub](https://help.github.com/en/enterprise/2.16/user/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) use [dependencies_https.rosinstall](install/dependencies_https.rosinstall).
```
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/polygon_coverage_planning.git
wstool init
wstool merge polygon_coverage_planning/install/dependencies.rosinstall
wstool update
```

Install [CGAL](https://www.cgal.org/) dependencies for catkinized package [cgal_catkin](https://www.github.com/ethz-asl/cgal_catkin.git).
```
sudo apt-get install libgmp-dev libmpfr-dev
```

Install [GTSP solver's](https://csee.essex.ac.uk/staff/dkarap/?page=publications&key=Gutin2009a) [Mono](https://www.mono-project.com/download/stable/) dependency.
```
sudo apt install gnupg ca-certificates
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
echo "deb https://download.mono-project.com/repo/ubuntu stable-bionic main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update
sudo apt install mono-devel
```

After installing all dependencies, build the workspace.
```
catkin build
```

## Getting Started
The package has a ROS interface for shortest path planning and coverage planning.
First source your workspace to execute any of the nodes.
```
source ~/catkin_ws/devel/setup.bash
```

### Coverage Planning
```
roslaunch polygon_coverage_ros coverage_planner.launch
```

The polygon can be set via
- ROS [service](polygon_coverage_msgs/srv/PolygonService.srv) call `rosservice call /coverage_planner/set_polygon`
- ROS [parameter](polygon_coverage_ros/launch/coverage_planner.launch) `/coverage_planner/polygon` or
- RVIZ Polygon Tool as in the video above.

The plan is generated via
- ROS [service](https://github.com/ethz-asl/mav_comm/blob/master/mav_planning_msgs/srv/PlannerService.srv) call 'rosservice call /coverage_planner/plan_path' or
- clicking start and goal points using the RVIZ clicked_point tool as in the video above.

### Euclidean Shortest Path Planning
```
roslaunch polygon_coverage_ros shortest_path_planner.launch
```

Setting the polygon and planning the path is the same as for Coverage Planning.

## Licensing
This repository is subject to GNU General Public License version 3 or later due to its dependencies.

The underlying (exact) geometric operations rely on [CGAL](https://www.cgal.org/license.html) which is restricted by GNU General Public License version 3 or later.

The underlying optimization the [memetic solver](https://csee.essex.ac.uk/staff/dkarap/?page=publications&key=Gutin2009a) presented in
```
Gutin, Gregory, and Daniel Karapetyan.
"A memetic algorithm for the generalized traveling salesman problem."
Natural Computing 9.1 (2010): 47-60.
```
is free of charge for non-commercial purposes only.

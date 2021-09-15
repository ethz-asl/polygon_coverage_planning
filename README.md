# polygon_coverage_planning
This package contains implementations to compute coverage patterns and shortest paths in general polygon with holes.
Please cite our [accompanying publication](https://arxiv.org/pdf/1907.09224) when using it.
```
Bähnemann, Rik, et al.
"Revisiting boustrophedon coverage path planning as a generalized traveling salesman problem."
Field and Service Robotics. Springer, Singapore, 2021.
```

![Coverage Planning in RVIZ](https://user-images.githubusercontent.com/11293852/61134221-70d18980-a4bf-11e9-87a7-d599b60c8dd2.gif)

[Watch the application video](https://youtu.be/u1UOqdJoK9s):

[![Watch the video](https://img.youtube.com/vi/u1UOqdJoK9s/sddefault.jpg)](https://youtu.be/u1UOqdJoK9s)

## Installation on Ubuntu 18.04 and ROS melodic
Install [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
Install [mono](https://www.mono-project.com/download/stable/#download-lin-ubuntu).

Create a workspace.
```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --extend /opt/ros/melodic
```

Download package dependencies from [dependencies.rosinstall](install/dependencies.rosinstall).<br>
**Note**: If you have not setup [SSH keys in GitHub](https://help.github.com/en/enterprise/2.16/user/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) use [dependencies_https.rosinstall](install/dependencies_https.rosinstall).
```
cd ~/catkin_ws/src
wstool init
wstool set --git polygon_coverage_planning git@github.com:ethz-asl/polygon_coverage_planning.git
wstool update
wstool merge polygon_coverage_planning/install/dependencies.rosinstall
wstool update
```

Install all [remaining dependencies](install/prepare-jenkins-slave.sh):
```
cd polygon_coverage_planning/install
./prepare-jenkins-slave.sh
```

Finally, build the workspace.
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
- ROS [service](polygon_coverage_msgs/srv/PlannerService.srv) call 'rosservice call /coverage_planner/plan_path' or
- clicking start and goal points using the RVIZ clicked_point tool as in the video above.

### Euclidean Shortest Path Planning
```
roslaunch polygon_coverage_ros shortest_path_planner.launch
```

Setting the polygon and planning the path is the same as for Coverage Planning.

## Licensing
This repository is subject to GNU General Public License version 3 or later due to its dependencies.

### CGAL dependencies
The underlying (exact) geometric operations rely on [CGAL 5.0.3](https://www.cgal.org/license.html) which is restricted by GNU General Public License version 3 or later.
In particular the dependencies are:
- Algebraic Foundations (LGPL)
- 2D and 3D Linear Geometry Kernel (LPGL)
- 2D Polygon (LPGL)
- Geometric Object Generators (LPGL)
- STL Extensions (LGPL)
- 2D Triangulation (GPL)
- 2D Regularized Boolean Set-Operations (GPL)
- 2D Straight Skeleton and Polygon Offsetting (GPL)
- 2D Arrangement (GPL), see also

```Fogel, Efi, Dan Halperin, and Ron Wein. CGAL arrangements and their applications: A step-by-step guide. Vol. 7. Springer Science & Business Media, 2012.```
```Bungiu, Francisc, et al. "Efficient computation of visibility polygons." arXiv preprint arXiv:1403.3905 (2014).```

### GTSP solver
The underlying optimization uses the [memetic solver](http://www.cs.nott.ac.uk/~pszdk/?page=publications&key=Gutin2009a) presented in
```
Gutin, Gregory, and Daniel Karapetyan.
"A memetic algorithm for the generalized traveling salesman problem."
Natural Computing 9.1 (2010): 47-60.
```
It is free of charge for non-commercial purposes only.

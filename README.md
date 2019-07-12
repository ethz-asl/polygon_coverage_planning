# UNDER CONSTRUCTION
# polygon_coverage_planning
This repository accompanies our publication
```
Bähnemann, Rik, et al.
"Revisiting Boustrophedon Coverage Path Planning as a Generalized Traveling Salesman Problem."
Field and Service Robotics. Springer, Cham, 2019.
```
It contains implementations to compute coverage patterns and shortest paths in general polygon with holes.
<!---

## Installation
Install [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Install catkin and wstool build dependencies.
```
sudo apt-get install python-wstool python-catkin-tools ros-kinetic-cmake-modules
```

Create a workspace.
```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config --extend /opt/ros/kinetic
```

Download package dependencies from [dependencies.rosinstall](install/dependencies.rosinstall).
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
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
sudo apt install apt-transport-https
echo "deb https://download.mono-project.com/repo/ubuntu stable-xenial main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update
sudo apt install mono-devel
```

After installing all dependencies, build the workspace.
```
catkin build
```

## Getting Started
The package has a ROS interface for shortest path planning and coverage planning.

### Coverage Planning
```
roslaunch polygon_coverage_planning_ros stripmap_planner_2d.launch  
```

In another terminal call
```
rosservice call /stripmap_planner_2d/plan_path "start_pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
start_velocity: {x: 0.0, y: 0.0, z: 0.0}
goal_pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
goal_velocity: {x: 0.0, y: 0.0, z: 0.0}
bounding_box: {x: 0.0, y: 0.0, z: 0.0}"
```

![An example coverage pattern.](https://user-images.githubusercontent.com/11293852/46402230-76fc3380-c6ff-11e8-8002-f8bdd512caf3.png)

### Euclidean Shortest Path Planning
```
roslaunch polygon_coverage_planning_ros shortest_path_planner_2d.launch
```

In another terminal call
```
rosservice call /shortest_path_planner_2d/plan_path "start_pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
start_velocity: {x: 0.0, y: 0.0, z: 0.0}
goal_pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 60.0, y: 110.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
goal_velocity: {x: 0.0, y: 0.0, z: 0.0}
bounding_box: {x: 0.0, y: 0.0, z: 0.0}"
```

![An example shortest path.](https://user-images.githubusercontent.com/11293852/46402328-b4f95780-c6ff-11e8-97c4-03d33a303ecd.png)

## Licensing
This repository is subject to GNU General Public License version 3 or later due to its dependencies.

The underlying (exact) geometric operations rely on [CGAL](https://www.cgal.org/license.html) which is restricted at most by GNU General Public License version 3 or later.

The underlying optimization the [memetic solver](https://csee.essex.ac.uk/staff/dkarap/?page=publications&key=Gutin2009a) presented in
```
Gutin, Gregory, and Daniel Karapetyan.
"A memetic algorithm for the generalized traveling salesman problem."
Natural Computing 9.1 (2010): 47-60.
```
is free of charge for non-commercial purposes only.

--->

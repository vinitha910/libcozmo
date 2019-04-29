# libcozmo [![Build Status](https://travis-ci.com/vinitha910/libcozmo.svg?branch=master)](https://travis-ci.com/vinitha910/libcozmo)

libcozmo is a C++ library for simulating and running [Cozmo](https://anki.com/en-us/cozmo) based on DART and AIKIDO. Additionally, this library has python bindings (cozmopy) for easier use with the [Cozmo SDK](http://cozmosdk.anki.com/docs/). Current tools allow you simulate the forklift movement. libcozmo currently only supports **Ubuntu 16.04** and is under heavy development. 

## Installation

Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and then install the following dependencies:
```
$ sudo add-apt-repository ppa:libccd-debs/ppa
$ sudo add-apt-repository ppa:fcl-debs/ppa
$ sudo add-apt-repository ppa:dartsim/ppa
$ sudo add-apt-repository ppa:personalrobotics/ppa
$ sudo apt-get update
$ sudo apt-get install cmake build-essential libboost-filesystem-dev libdart6-optimizer-nlopt-dev libdart6-utils-dev libdart6-utils-urdf-dev libmicrohttpd-dev libompl-dev libtinyxml2-dev libyaml-cpp-dev pr-control-msgs
$ sudo apt-get install ros-kinetic-pybind11-catkin
```

Checkout and build [aikido](https://github.com/personalrobotics/aikido.git) from source. You can automate the checkout and build by following [development environment](https://www.personalrobotics.ri.cmu.edu/software/development-environment)
instructions with this `.rosinstall` file:
```yaml
- git:
    local-name: aikido
    uri: https://github.com/personalrobotics/aikido.git
    version: master
- git:
    local-name: libcozmo
    uri: https://github.com/vinitha910/libcozmo
    version: cozmopy
```

## Usage
To load Cozmo into the Rviz viewer in a catkin and ros environment, run the following commands:
```shell
$ catkin build libcozmo
$ . devel/setup.bash
$ cd libcozmo
$ screen -S roscore
$ roscore
$ <CTRL><A>+<D>
$ screen -S rviz
$ . devel/setup.bash
$ rviz
$ <CTRL><A>+<D>
$ rosrun libcozmo rviz_example `catkin locate -s libcozmo`/src/cozmo_description/meshes
```
where `MESH_DIR` is the **full path** to the `libcozmo/cozmo_description/meshes` folder. After all the commands are run, subscribe to the InteractiveMarker topic in Rviz. Cozmo should now appear in the viewer.

This script allows you to enter angles (in radians) for the forklift position; the movement will be reflected by the robot in the viewer.

To load Cozmo in the DART viewer in a non-catkin/ros environment, run:
```shell
$ rosrun libcozmo dart_example `catkin locate -s libcozmo`/src/cozmo_description/meshes`
```

## Trajectory Execution in Simulation

A sample script has been provided to show how to simulate trajectory execution in the Rviz viewer. To run this script, run the following commands:
```shell
$ screen -S roscore
$ roscore
$ <CTRL><A>+<D>
$ screen -S rviz
$ . devel/setup.bash
$ rviz
$ <CTRL><A>+<D>
$ rosrun libcozmo execute_traj `catkin locate -s libcozmo`/src/cozmo_description/meshes
```

A trajectory is defined by a set of waypoints. First, define waypoints at specific times:
```shell
libcozmo::Waypoint w1;
w1.x = X_POSITION;
w1.y = Y_POSITION;
w1.th = ROTATION_THETA;
w1.t = TIME;
```

Pass in an `std::vector` of waypoints to the `createInterpolatedTraj` function to create an interpolated trajectory. Pass this trajectory and a period into the `executeTrajectory` function to execute the trajectory.

## License
libcozmo is licensed under a BSD license. See [LICENSE](https://github.com/personalrobotics/libcozmo/blob/master/LICENSE) for more information.

## Author/Acknowledgements
libcozmo is developed by Vinitha Ranganeni ([**@vinitha910**](https://github.com/vinitha910)) at the [Human-Centered Robotics Lab](https://hcrlab.cs.washington.edu/) in [University of Washington](https://www.cs.washington.edu/). Initial development began at the [Personal Robotics Lab](https://personalrobotics.ri.cmu.edu/) in the [Robotics Institute](http://ri.cmu.edu/) at [Carnegie Mellon University](http://www.cmu.edu/). I would like to thank Clint Liddick ([**@ClintLiddick**](https://github.com/ClintLiddick)) and J.S. Lee ([**@jslee02**](https://github.com/jslee02)) for their assistance in developing libcozmo and Ariana Keeling for her assistance in developing the SolidWorks model of Cozmo.

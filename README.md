# libcozmo [![Build Status](https://travis-ci.com/vinitha910/libcozmo.svg?branch=master)](https://travis-ci.com/vinitha910/libcozmo)

libcozmo is a C++ library for simulating and running [Cozmo](https://anki.com/en-us/cozmo) based on DART and AIKIDO. Additionally, this library has python bindings (cozmopy) for easier use with the [Cozmo SDK](http://cozmosdk.anki.com/docs/). Current tools allow you simulate the forklift movement. libcozmo currently only supports **Ubuntu 16.04** and is under heavy development. 

## Installation

Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and then install the following dependencies:
```shell
$ sudo add-apt-repository ppa:personalrobotics/ppa
$ sudo apt-get update
# DART Dependencies
$ sudo apt-get install cmake build-essential libboost-filesystem-dev libmicrohttpd-dev libompl-dev libtinyxml2-dev libyaml-cpp-dev
$ sudo apt-get install libopenscenegraph-dev
$ sudo apt-get install libnlopt-dev coinor-libipopt-dev libbullet-dev libode-dev
$ sudo apt-get install libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev doxygen
# libcozmo Dependencies
$ sudo apt-get install ros-kinetic-rospy ros-kinetic-pybind11-catkin ros-kinetic-octomap-ros libeigen3-dev python-catkin-tools python-catkin-pkg
$ sudo apt-get install python3-yaml
$ sudo pip3 install rospkg catkin_pkg empy
# roscpp_initializer Dependencies
$ sudo apt-get install libboost-all-dev python3-dev libpython3-dev 
```

Install [DART](https://dartsim.github.io/install_dart_on_ubuntu.html#build-and-install-dart) from source:
```shell
$ git clone git://github.com/dartsim/dart.git
$ cd dart
$ git checkout tags/v6.8.2
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
$ sudo make install
```

Checkout and build [aikido](https://github.com/personalrobotics/aikido.git) from source. You can automate the checkout and build by following [development environment](https://personalrobotics.cs.washington.edu/software/development-environment)
instructions with this `.rosinstall` file:
```yaml
- git:
    local-name: aikido
    uri: https://github.com/personalrobotics/aikido.git
    version: master
- git:
    local-name: aikidopy
    uri: https://github.com/vinitha910/aikidopy.git
    version: master
- git:
    local-name: roscpp_initializer
    uri: https://github.com/vinitha910/roscpp_initializer.git
    version: master
- git:
    local-name: libcozmo
    uri: https://github.com/vinitha910/libcozmo
    version: master
```
Put the `.rosinstall` file in the src directory of your cozmo workspace and run `rosinstall .` in the same directory.
Now run `catkin build` to build your workspace. The build might fail if your `PYTHONPATH` is not up-to-date. Run `echo $PYTHONPATH` if you see something like this (i.e. no path to `python3`)
```
<PATH_TO_COZMO_WORKSPACE>/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages
```
you'll need to add `export PYTHONPATH="<PATH_TO_COZMO_WORKSPACE>/devel/lib/python3/dist-packages":"${PYTHONPATH}"` to your `.bashrc`

If you have multiple versions of python follow these [instructions](https://github.com/vinitha910/roscpp_initializer#handling-multiple-python-versions) so that you can build `roscpp_initializer`.

Once you finish building, source the `setup.bash` file by running `source <PATH_TO_COZMO_WORKSPACE>/devel/setup.bash`
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
After all the commands are run, subscribe to the InteractiveMarker topic in Rviz. Cozmo should now appear in the viewer.

This script allows you to enter angles (in radians) for the forklift position; the movement will be reflected by the robot in the viewer.

To load Cozmo in the DART viewer in a non-catkin/ros environment, run:
```shell
$ rosrun libcozmo dart_example `catkin locate -s libcozmo`/src/cozmo_description/meshes
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
You should see Cozmo moving in the shape of a square. 

A trajectory is defined by a set of waypoints. First, define waypoints at specific times:
```shell
libcozmo::Waypoint w1;
w1.x = X_POSITION;
w1.y = Y_POSITION;
w1.th = ROTATION_THETA;
w1.t = TIME;
```

Pass in an `std::vector` of waypoints to the `createInterpolatedTraj` function to create an interpolated trajectory. Pass this trajectory and a period into the `executeTrajectory` function to execute the trajectory.

## cozmopy 

`libcozmo` additionally comes with python bindings. After the package is built you should be able to load `cozmopy` in python:

```shell
$ python
>>> import cozmopy
```
`cozmopy` depends on `aikidopy` and `roscpp_initializer`; you should make sure you can load both packages in python as well.

A python sample script for trajectory execution in similation has been provided as well. Follow the instructions in the previous section but replace the last command with

```shell
$ rosrun libcozmo execute_traj.py `catkin locate -s libcozmo`/src/cozmo_description/meshes 
```

Similarily, to run the forklift simulation, run:

```shell
rosrun libcozmo rviz_example.py `catkin locate -s libcozmo`/src/cozmo_description/meshes 
```

Before running the commands, make sure the python script is executable. If it is not, run `chmod +x <SCRIPT_NAME>` in the appropiate directory. 

## License
libcozmo is licensed under a BSD license. See [LICENSE](https://github.com/vinitha910/libcozmo/blob/master/LICENSE) for more information.

## Author/Acknowledgements
libcozmo is developed by Vinitha Ranganeni ([**@vinitha910**](https://github.com/vinitha910)) at the [Human-Centered Robotics Lab](https://hcrlab.cs.washington.edu/) in [University of Washington](https://www.cs.washington.edu/). Initial development began at the [Personal Robotics Lab](https://personalrobotics.ri.cmu.edu/) in the [Robotics Institute](http://ri.cmu.edu/) at [Carnegie Mellon University](http://www.cmu.edu/). I would like to thank Clint Liddick ([**@ClintLiddick**](https://github.com/ClintLiddick)) and J.S. Lee ([**@jslee02**](https://github.com/jslee02)) for their assistance in developing libcozmo and Ariana Keeling for her assistance in developing the SolidWorks model of Cozmo.

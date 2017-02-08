# libcozmo
libcozmo is a C++ library for simulating and running [Cozmo](https://anki.com/en-us/cozmo) based on DART and AIKIDO.
Current tools allow you simulate the forklift movement and move Cozmo to a specified pose.

## Installation
Checkout and build this package, [aikido](https://github.com/personalrobotics/aikido.git) from source, and install [DART](http://dartsim.github.io/) (version 6.0 or above). You
can automate the checkout and build by following [development environment]
(https://www.personalrobotics.ri.cmu.edu/software/development-environment)
instructions with this `.rosinstall` file:
```yaml
- git:
    local-name: aikido
    uri: https://github.com/personalrobotics/aikido.git
    version: xenial_fixes
- git:
    local-name: cozmo_description
    uri: https://github.com/personalrobotics/cozmo_description.git
    version: master
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
$ `catkin locate -b libcozmo`/rviz_example `catkin locate -s libcozmo`/meshes
```
After all the commands are run, subscribe to the InteractiveMarker `dart_markers` topic in Rviz. Cozmo should now appear in the viewer.

This script allows you to enter angles (in radians) for the forklift position; the movement will be reflected by the robot in the viewer.

Similarily, to run the script that moves Cozmo to the specified pose, run the following command: 
```shell
$ `catkin locate -b libcozmo`/go_to_pose_example `catkin locate -s libcozmo`/meshes
```
NOTE: You must be connected to Cozmo in SDK Mode to run this script; the script does not simulate the movement in the viewer. For instructions on how to connect to Cozmo in SDK Mode [click here](http://cozmosdk.anki.com/docs/initial.html).

To load Cozmo in the DART viewer in a non-catkin/ros environment, run the following commands:
```shell
$ cd libcozmo
$ mkdir build
$ cd build
$ cmake .. -DCOZMO_BUILD_RVIZ_EXAMPLE=OFF # e.g. if ros/aikido not available 
$ make
$ ./dart_example `pwd`/../meshes
```

## License
libcozmo is licensed under a BSD license. See [LICENSE](https://github.com/personalrobotics/libcozmo/blob/master/LICENSE) for more information.

## Author/Acknowledgements
libcozmo is developed by Vinitha Ranganeni ([**@vinitha910**](https://github.com/vinitha910)) at the [Personal Robotics Lab](https://personalrobotics.ri.cmu.edu/) in the [Robotics Institute](http://ri.cmu.edu/) at [Carnegie Mellon University](http://www.cmu.edu/). I would like to thank Clint Liddick ([**@ClintLiddick**](https://github.com/ClintLiddick)) and J.S. Lee ([**@jslee02**](https://github.com/jslee02)) for their assistance in developing libcozmo and Ariana Keeling for her assistance in developing the SolidWorks model of Cozmo. 
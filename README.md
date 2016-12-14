# libcozmo
libcozmo is a C++ library for simulating and running Cozmo based on DART and AIKIDO.
Current tools allow you simulate the forklift movement.

## Installation
Checkout and build this package, [DART](https://github.com/dartsim/dart.git) (version 6.0 or above)
and [aikido](https://github.com/personalrobotics/aikido.git) from source. You
can automate the checkout and build by following [development environment]
(https://www.personalrobotics.ri.cmu.edu/software/development-environment)
instructions with this `.rosinstall` file:
```yaml
- git:
    local-name: aikido
    uri: https://github.com/personalrobotics/aikido.git
    version: xenial_fixes
- git:
    local-name: dart
    uri: https://github.com/dartsim/dart.git
    version: release-6.0
- git:
    local-name: cozmo_description
    uri: https://github.com/personalrobotics/cozmo_description.git
    version: master
```
## Usage
To load Cozmo into the Rviz viewer, run the following commands:
```shell
$ cd libcozmo
$ screen -S roscore
$ roscore
$ <CTRL><A>+<D>
$ screen -S rviz
$ . devel/setup.bash
$ rviz
$ <CTRL><A>+<D>
$ rosrun cozmo_description rviz_example MESH_DIR
```
where `MESH_DIR` is the path to the mesh directory. After all the commands are run, subscribe to the InteractiveMarker topic in Rviz. Cozmo should now appear in the viewer.

Similarily, to load Cozmo the in DART viewer, run the following command (currently not working):
```shell
$ rosrun cozmo_description dart_example MESH_DIR
```

These scripts allows you to enter angles (in radians) for the forklift position and the movement will be reflected by the robot in the viewer.
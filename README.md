# LibCozmo DART Library
LibCozmo is C++ library that creates and loads the Cozmo, a robot developed by Anki, into the DART Simulator.
Current tools allow you simulate the forklift movement.

## Installation
Checkout and build this package, DART (version 6.0 or above)
and [aikido](https://github.com/personalrobotics/aikido.git) from source. You
can automate the checkout and build by following [development environment]
(https://www.personalrobotics.ri.cmu.edu/software/development-environment)
instructions with this `.rosinstall` file:
```yaml
- git:
    local-name: aikido
    uri: https://github.com/personalrobotics/aikido.git
    version: master
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
To load Cozmo into the rviz viewer, run the following command:
```shell
$ rosrun cozmo_description createCozmo MESH_DIR
```
where `MESH_DIR` is the path to the mesh directory. This script allows you
to enter angles (in radians) for the forklift position and the movement will
be reflected by the robot in the viewer.

Similarily, to load Cozmo the in DART viewer, run the following command (currently not working):
To load Cozmo into the rviz viewer, run the following command:
```shell
$ rosrun cozmo_description dart_example MESH_DIR
```
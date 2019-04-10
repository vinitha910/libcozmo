# libcozmo
libcozmo is a C++ library for simulating and running [Cozmo](https://anki.com/en-us/cozmo) based on DART and AIKIDO.
Current tools allow you simulate the forklift movement.

## Installation
Checkout and build this package, [DART](https://github.com/dartsim/dart.git) (version 6.0 or above)
and [aikido](https://github.com/personalrobotics/aikido.git) from source. You
can automate the checkout and build by following [development environment](https://www.personalrobotics.ri.cmu.edu/software/development-environment)
instructions with this `.rosinstall` file:
```yaml
- git:
    local-name: aikido
    uri: https://github.com/personalrobotics/aikido.git
    version: master
- git:
    local-name: dart
    uri: https://github.com/dartsim/dart.git
    version: v6.7.0
- git:
    local-name: libcozmo
    uri: https://github.com/vinitha910/libcozmo
    version: cozmopy
```

Once the dependencies are install, you can build libcozmo using CMake:
```bash
$ mkdir build
$ cd build
$ cmake ..
$ make 
$ sudo make install
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
$ rosrun libcozmo rviz_example MESH_DIR
```
where `MESH_DIR` is the **full path** to the `libcozmo/cozmo_description/meshes` folder. After all the commands are run, subscribe to the InteractiveMarker topic in Rviz. Cozmo should now appear in the viewer.

This script allows you to enter angles (in radians) for the forklift position; the movement will be reflected by the robot in the viewer.

Similarily, to load Cozmo the in DART viewer, run the following command:
```shell
$ rosrun libcozmo dart_example MESH_DIR
```

## License
libcozmo is licensed under a BSD license. See [LICENSE](https://github.com/personalrobotics/libcozmo/blob/master/LICENSE) for more information.

## Acknowledgements
I would like to thank Clint Liddick ([**@ClintLiddick**](https://github.com/ClintLiddick)) and J.S. Lee ([**@jslee02**](https://github.com/jslee02)) for their assistance in developing libcozmo and Ariana Keeling for her assistance in developing the SolidWorks model of Cozmo. 
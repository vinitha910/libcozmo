cmake_minimum_required(VERSION 2.8.12)
project(cozmo_description)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY "meshes"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)

add_library(${PROJECT_NAME}
  src/cozmo.cpp
  src/createCozmo.cpp
)
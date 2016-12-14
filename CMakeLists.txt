cmake_minimum_required(VERSION 2.8.12)
project(cozmo_description)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()

find_package(aikido REQUIRED COMPONENTS rviz)

find_package(Eigen3 REQUIRED)
find_package(DART REQUIRED gui collision-bullet)

install(DIRECTORY "meshes"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)

add_library(cozmo
  src/cozmo.cpp
)

target_include_directories(cozmo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
 
target_include_directories(cozmo PRIVATE
  ${DART_INCLUDE_DIRS}
)

target_include_directories(cozmo PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(cozmo PUBLIC
  ${DART_LIBRARIES}
)

add_executable(createCozmo
  src/createCozmo.cpp
)

add_executable(dart_example
  src/dart_example.cpp
)  

target_include_directories(createCozmo PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
)

target_link_libraries(createCozmo
  cozmo
  ${catkin_LIBRARIES}
  ${aikido_LIBRARIES}
)

target_link_libraries(dart_example
  cozmo
)

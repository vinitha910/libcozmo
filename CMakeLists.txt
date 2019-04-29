cmake_minimum_required(VERSION 2.8.12)
project(libcozmo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp pybind11_catkin)
find_package(aikido REQUIRED COMPONENTS rviz)
find_package(Eigen3 REQUIRED)
find_package(DART REQUIRED gui collision-bullet)

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS pybind11_catkin
)

add_library(cozmo SHARED
  src/cozmo_description/cozmo.cpp
)

target_include_directories(cozmo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(cozmo PRIVATE ${DART_INCLUDE_DIRS})
target_include_directories(cozmo PUBLIC ${EIGEN3_INCLUDE_DIR})

target_link_libraries(cozmo PUBLIC ${DART_LIBRARIES})

add_executable(rviz_example src/examples/rviz_example.cpp)
add_executable(dart_example src/examples/dart_example.cpp)

target_include_directories(rviz_example PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
)

target_link_libraries(dart_example cozmo)
target_link_libraries(rviz_example
  cozmo
  ${catkin_LIBRARIES}
  ${aikido_LIBRARIES}
)

#################### PYBIND ##############################

include_directories(${catkin_INCLUDE_DIRS})
pybind_add_module(cozmopy MODULE
    src/cozmopy/cozmopy.cpp
)

target_link_libraries(cozmopy PUBLIC
  cozmo
  ${DART_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

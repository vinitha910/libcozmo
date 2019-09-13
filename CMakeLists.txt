cmake_minimum_required(VERSION 3.0)
project(libcozmo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  pybind11_catkin 
  std_msgs 
  message_generation 
  geometry_msgs 
)

find_package(Eigen3 REQUIRED)
find_package(DART 6.8.2 REQUIRED COMPONENTS gui collision-bullet)
find_package(aikido REQUIRED COMPONENTS statespace trajectory distance control rviz)
find_package(aikidopy REQUIRED COMPONENTS libaikidopy)

add_message_files(
  DIRECTORY msg
  FILES
  ForkLift.msg
  ObjectOrientedAction.msg
  ActionMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
      pybind11_catkin 
      message_runtime  
      geometry_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_library(cozmo SHARED
  src/cozmo_description/cozmo.cpp
  src/actionspace/ObjectOrientedActionSpace.cpp
  src/actionspace/GenericActionSpace.cpp
  src/statespace/SE2.cpp
  src/distance/SE2.cpp
  src/distance/translation.cpp
  src/distance/orientation.cpp
)

target_include_directories(cozmo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(
  cozmo 
  PUBLIC 
    ${EIGEN3_INCLUDE_DIR}
  PRIVATE 
    ${DART_INCLUDE_DIRS} 
    ${aikido_INCLUDE_DIRS}
)
target_link_libraries(
  cozmo 
  PUBLIC 
    ${DART_LIBRARIES} 
    ${aikido_LIBRARIES} 
)

################################################################################
# TESTS       
################################################################################
set(TEST_LIBS cozmo ${catkin_LIBRARIES} ${aikido_LIBRARIES})

catkin_add_gtest(test_libcozmo tests/statespace/test_statespace.cpp)
target_link_libraries(test_libcozmo ${TEST_LIBS})

catkin_add_gtest(test_generic_action_space tests/actionspace/test_generic_action_space.cpp )
target_link_libraries(test_generic_action_space ${TEST_LIBS})

catkin_add_gtest(test_angle_normalization tests/utils/test_angle_normalization.cpp)
target_link_libraries(test_angle_normalization ${TEST_LIBS})

catkin_add_gtest(test_object_oriented_action_space tests/actionspace/test_object_oriented_action_space.cpp)
target_link_libraries(test_object_oriented_action_space
    cozmo
)

catkin_add_gtest(test_euclidean tests/utils/test_euclidean_distance.cpp)
target_link_libraries(test_euclidean ${TEST_LIBS})

catkin_add_gtest(test_linspace tests/utils/test_linspace.cpp)
target_link_libraries(test_linspace ${TEST_LIBS})

catkin_add_gtest(test_distance tests/distance/test_distance.cpp)
target_link_libraries(test_distance ${TEST_LIBS})

################################################################################
# EXECUTABLES       
################################################################################


add_executable(rviz_example src/examples/rviz_example.cpp)
target_include_directories(rviz_example PRIVATE
  ${aikido_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)
target_link_libraries(rviz_example
  cozmo
  ${aikido_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_executable(dart_example src/examples/dart_example.cpp)
target_include_directories(dart_example PRIVATE 
  ${aikido_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)
target_link_libraries(dart_example cozmo ${aikido_LIBRARIES})

add_executable(execute_traj src/examples/execute_traj.cpp)
target_include_directories(execute_traj PRIVATE
  ${aikido_INCLUDE_DIRS}
)
target_link_libraries(execute_traj
  cozmo
  ${aikido_LIBRARIES}
)

################################################################################
# PYBIND 
################################################################################

include_directories(${catkin_INCLUDE_DIRS})

pybind_add_module(cozmopy MODULE
    src/cozmopy/cozmopy.cpp
)

target_link_libraries(cozmopy PUBLIC
  cozmo
  ${libaikidopy_LIBRARIES}
  ${DART_LIBRARIES}
)

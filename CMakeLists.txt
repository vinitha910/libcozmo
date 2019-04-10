cmake_minimum_required(VERSION 2.8.12)
project(libcozmo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)
set(INCLUDE_INSTALL_DIR "include")
set(LIBRARY_INSTALL_DIR "lib")
set(CONFIG_INSTALL_DIR "${LIBRARY_INSTALL_DIR}/${PROJECT_NAME}/cmake")

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(aikido REQUIRED COMPONENTS rviz)
find_package(Eigen3 REQUIRED)
find_package(DART REQUIRED gui collision-bullet)

#install(DIRECTORY "meshes"
#    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
#)

add_library(cozmo SHARED
  src/cozmo_description/cozmo.cpp
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

add_executable(rviz_example
  src/examples/rviz_example.cpp
)

add_executable(dart_example
  src/examples/dart_example.cpp
)  

target_include_directories(rviz_example PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
)

target_link_libraries(rviz_example
  cozmo
  ${catkin_LIBRARIES}
  ${aikido_LIBRARIES}
)

target_link_libraries(dart_example
  cozmo
)

install(DIRECTORY "include/"
  DESTINATION "${INCLUDE_INSTALL_DIR}")

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/${PROJECT_NAME}")

install(DIRECTORY src/include/cozmo_description DESTINATION include
  FILES_MATCHING PATTERN "*.h")
install(TARGETS cozmo
  LIBRARY DESTINATION lib)


cmake_minimum_required(VERSION 2.8.12)
project(libcozmo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(COZMO_BUILD_RVIZ_EXAMPLE "Build the RViz viewer integration example" ON)

find_package(Eigen3 REQUIRED)
find_package(DART REQUIRED gui collision-bullet)

add_library(cozmo SHARED
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


add_executable(dart_example
  src/dart_example.cpp
)

target_link_libraries(dart_example
  cozmo
)


if(COZMO_BUILD_RVIZ_EXAMPLE)
  find_package(roscpp)
  find_package(aikido REQUIRED COMPONENTS rviz)
  find_package(PythonLibs REQUIRED)
  
  add_executable(rviz_example
    src/rviz_example.cpp
  )

  target_include_directories(rviz_example PRIVATE
    ${aikido_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
  )

  target_link_libraries(rviz_example
    cozmo
    ${aikido_LIBRARIES}
    ${roscpp_LIBRARIES}
  )

  add_executable(sim_example
    src/sim_example.cpp
  )

  target_include_directories(sim_example PRIVATE
    ${aikido_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS} 
  )

  target_include_directories(sim_example PUBLIC
    ${PYTHON_INCLUDE_DIRS}  
  )

  target_link_libraries(sim_example
    cozmo
    ${aikido_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${PYTHON_INCLUDE_DIRS}  
  )


endif()


#================================================================================
# Installation
#

install(TARGETS cozmo EXPORT cozmoConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the install directory
install(EXPORT cozmoConfig DESTINATION share/libcozmo/cmake)

# This makes the project importable from the build directory
export(TARGETS cozmo FILE cozmoConfig.cmake)

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/libcozmo"
)

install(DIRECTORY meshes
  DESTINATION "share/libcozmo"
)

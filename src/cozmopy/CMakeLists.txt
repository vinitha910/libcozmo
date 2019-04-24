cmake_minimum_required(VERSION 3.0)
project(cozmopy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(PYTHON_INCLUDE_DIRS "/usr/include/python2.7")

add_subdirectory(pybind11)
#find_package(pybind11 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED )
find_package(PythonLibs 2.7 REQUIRED )
find_package(Eigen3 REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

pybind11_add_module(cozmopy MODULE
    cozmopy.cpp
)

target_include_directories(cozmopy
  PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
  PRIVATE
    ${DART_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(cozmopy PUBLIC
  cozmo
  ${DART_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

set_target_properties(cozmopy
  PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    DEBUG_POSTFIX ""
)

install(TARGETS cozmopy
  LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
)

cmake_minimum_required(VERSION 2.8.12)
project(cozmopy)

find_package(pybind11 REQUIRED)
if(NOT pybind11_FOUND)
  message(STATUS "BUILD_PYTHON is ON, but failed to find pybind11 >= "
    "2.2.0. Disabling cozmopy."
  )
  return()
endif()

execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

pybind11_add_module(cozmopy
  MODULE
  src/cozmopy/cozmo_py.cpp
  src/cozmopy/cozmopy.cpp
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
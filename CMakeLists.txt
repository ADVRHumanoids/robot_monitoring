cmake_minimum_required(VERSION 3.0)
project(robot_monitoring VERSION 1.0)

# std c++14 flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CATKIN_BUILD_BINARY_PACKAGE TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    roslib
    )

###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES robot_monitoring_tools
    #  CATKIN_DEPENDS roscpp rviz
    #  DEPENDS system_lib
    SKIP_CMAKE_CONFIG_GENERATION
    SKIP_PKG_CONFIG_GENERATION
    )

add_subdirectory(src/chart/)
add_subdirectory(src/joint_state_gui/)
add_subdirectory(src/joint_sliders/)

include(cmake/GenerateDeb.cmake)

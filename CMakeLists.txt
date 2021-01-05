cmake_minimum_required(VERSION 3.0)
project(robot_monitoring VERSION 1.0)

# std c++14 flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# avoid installing catkin-related files
set(CATKIN_BUILD_BINARY_PACKAGE TRUE)

# fmt header only mode to avoid linking
find_package(fmt REQUIRED)

# find required packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    roslib
    )

# set qt to automatically run MOC and RCC pre-compilers
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# give option to user to give custom qt path
set(QT5_PATH "" CACHE PATH "Path to QT")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${QT5_PATH}")

# find qt
find_package(Qt5UiTools REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Charts  QUIET)
find_package(Qt5PrintSupport REQUIRED)


# robotinterfaceros is needed just to construct
# xbotinterface from ros parameters (could be avoided)
find_package(RobotInterfaceROS REQUIRED)
include_directories(include/
    src/
	${catkin_INCLUDE_DIRS}
    ${RobotInterfaceROS_INCLUDE_DIRS})

# generate catkin package, but don't let it export
# any target
catkin_package(
    SKIP_CMAKE_CONFIG_GENERATION
    SKIP_PKG_CONFIG_GENERATION
    )

add_subdirectory(src/robot_monitoring/)
add_subdirectory(src/chart/)
add_subdirectory(src/joint_state_gui/)
add_subdirectory(src/joint_sliders/)
add_subdirectory(src/imu)
add_subdirectory(src/ft)
add_subdirectory(src/rviz/)
add_subdirectory(src/qcustomplot)

include(cmake/GenerateDeb.cmake)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(xbot_rob_mon_plugin_imu SHARED
    imu_widget.cpp
    imu_ui_resources.qrc)

target_link_libraries(xbot_rob_mon_plugin_imu robot_monitoring::robot_monitoring)

target_include_directories(xbot_rob_mon_plugin_imu
    PRIVATE
    ${catkin_INCLUDE_DIRS})

install(TARGETS xbot_rob_mon_plugin_imu
    DESTINATION lib)

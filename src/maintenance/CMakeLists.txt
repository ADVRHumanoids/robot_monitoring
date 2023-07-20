set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(xbot_rob_mon_plugin_maintenance SHARED
    maintenance_widget.cpp
    maintenance_ui_resources.qrc)

target_link_libraries(xbot_rob_mon_plugin_maintenance robot_monitoring::robot_monitoring)

target_include_directories(xbot_rob_mon_plugin_maintenance
    PRIVATE
    ${catkin_INCLUDE_DIRS})

install(TARGETS xbot_rob_mon_plugin_maintenance
    DESTINATION lib)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(rviz REQUIRED)

add_library(xbot_rob_mon_plugin_rviz SHARED
    rviz_widget.cpp)

target_link_libraries(xbot_rob_mon_plugin_rviz
    robot_monitoring::robot_monitoring
    ${rviz_LIBRARIES})

target_include_directories(xbot_rob_mon_plugin_rviz
    PRIVATE
    ${rviz_INCLUDE_DIRS})

install(TARGETS xbot_rob_mon_plugin_rviz
    DESTINATION lib)

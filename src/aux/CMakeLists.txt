set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(xbot_rob_mon_plugin_aux SHARED
    aux_widget.cpp
    aux_ui_resources.qrc)

target_link_libraries(xbot_rob_mon_plugin_aux robot_monitoring::robot_monitoring)

target_include_directories(xbot_rob_mon_plugin_aux
    PRIVATE
    ${catkin_INCLUDE_DIRS})

install(TARGETS xbot_rob_mon_plugin_aux
    DESTINATION lib)

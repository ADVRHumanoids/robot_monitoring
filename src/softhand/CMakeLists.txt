set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(xbot_rob_mon_plugin_softhand SHARED
    softhand_widget.cpp
    softhand_ui_resources.qrc)

target_link_libraries(xbot_rob_mon_plugin_softhand robot_monitoring::robot_monitoring)

target_include_directories(xbot_rob_mon_plugin_softhand
    PRIVATE
    ${catkin_INCLUDE_DIRS})

install(TARGETS xbot_rob_mon_plugin_softhand
    DESTINATION lib)

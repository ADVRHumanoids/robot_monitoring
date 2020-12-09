set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(xbot_rob_mon_plugin_ft SHARED
    ft_widget.cpp
    ft_ui_resources.qrc)

target_link_libraries(xbot_rob_mon_plugin_ft robot_monitoring::robot_monitoring)

install(TARGETS xbot_rob_mon_plugin_ft
    DESTINATION lib)

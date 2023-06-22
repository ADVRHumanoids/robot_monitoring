set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(xbot_rob_mon_plugin_automatica SHARED
    automatica_widget.cpp
    )

target_link_libraries(xbot_rob_mon_plugin_automatica robot_monitoring::robot_monitoring)

target_include_directories(xbot_rob_mon_plugin_automatica
    PRIVATE
    ${catkin_INCLUDE_DIRS})

install(TARGETS xbot_rob_mon_plugin_automatica
    DESTINATION lib)

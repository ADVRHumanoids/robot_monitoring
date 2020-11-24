set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(fmt REQUIRED)

add_library(robot_monitoring SHARED
    custom_qt_widget.cpp
    )
    
add_library(robot_monitoring::robot_monitoring ALIAS robot_monitoring)

target_link_libraries(robot_monitoring PUBLIC fmt::fmt-header-only Qt5::Widgets Qt5::UiTools)

install(TARGETS robot_monitoring
    DESTINATION lib)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(robot_monitoring SHARED
    custom_qt_widget.cpp
    context.cpp
    )
    
add_library(robot_monitoring::robot_monitoring ALIAS robot_monitoring)

target_link_libraries(robot_monitoring
    PUBLIC
    Qt5::Widgets
    Qt5::UiTools)

if(TARGET fmt::fmt-header-only)
    target_link_libraries(robot_monitoring
        PUBLIC fmt::fmt-header-only
        )
else()
    target_link_libraries(robot_monitoring
        PUBLIC fmt::fmt
        )
endif()

install(TARGETS robot_monitoring
    DESTINATION lib)

install(FILES custom_qt_widget.h custom_qt_widget_impl.h 
DESTINATION include/robot_monitoring)
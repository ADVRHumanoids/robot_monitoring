
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5UiTools REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick   REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp urdf)

include_directories(../../include)

add_library(joint_bar_ui STATIC
    bar_plot_widget.cpp
    load_plugin_wid.cpp
    top_right_tab.cpp
    joint_bar_widget.cpp
    joint_state_widget.cpp
    joint_monitor_widget.cpp
    circle_widget.cpp
    xbot2_wid.cpp
    xbot2_plugin_wid.cpp
    xbot2_status_wid.cpp
    ui_resources.qrc)

target_link_libraries(joint_bar_ui
    sliders_widget
    chart_widget
    robot_monitoring
    qcustom_chart
    Qt5::Widgets
    Qt5::UiTools XBotInterface::XBotInterface ${catkin_LIBRARIES} yaml-cpp)

add_executable(xbot2-gui ui_main.cpp)
target_link_libraries(xbot2-gui joint_bar_ui)

# Mark executables and/or libraries for installation
install(TARGETS xbot2-gui
    RUNTIME DESTINATION bin
    )

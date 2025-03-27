
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5UiTools REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick   REQUIRED)
find_package(xbot2_interface REQUIRED)
#find_package(ec_srvs REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(xbot_msgs REQUIRED) 
find_package(std_srvs REQUIRED) 
find_package(urdf REQUIRED)

include_directories(../../include)

add_library(joint_bar_ui STATIC
    #bar_plot_widget.cpp
    #bringup_widget.cpp
    #load_plugin_wid.cpp
    #top_right_tab.cpp
    #joint_bar_widget.cpp
    #joint_state_widget.cpp
    joint_monitor_widget.cpp
    #circle_widget.cpp
    xbot2_wid.cpp
    xbot2_plugin_wid.cpp
    xbot2_status_wid.cpp
    ui_resources.qrc)

target_link_libraries(joint_bar_ui
    sliders_widget
    chart_widget
    robot_monitoring::robot_monitoring
    qcustom_chart
    Qt5::Widgets
    Qt5::UiTools
    xbot2_interface::xbot2_interface
    ${xbot_msgs_TARGETS}
    ${std_srvs_TARGETS}
	rclcpp::rclcpp
    urdf::urdf
    yaml-cpp)

target_include_directories(joint_bar_ui
    PRIVATE ${ec_srvs_INCLUDE_DIRS}
    )

add_executable(xbot2-gui ui_main.cpp)
target_link_libraries(xbot2-gui joint_bar_ui uuid)

# Mark executables and/or libraries for installation
install(TARGETS xbot2-gui
    RUNTIME DESTINATION bin
    )

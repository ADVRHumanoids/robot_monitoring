
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5UiTools ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Widgets ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Quick ${MINIMUM_QT_VERSION} REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp urdf)

include_directories(../../include)

add_library(joint_bar_ui STATIC
    bar_plot_widget.cpp
    joint_bar_widget.cpp
    joint_state_widget.cpp
    joint_monitor_widget.cpp
    circle_widget.cpp
    ui_resources.qrc)

target_link_libraries(joint_bar_ui sliders_widget chart_widget
    Qt5::Widgets Qt5::UiTools XBotInterface::XBotInterface ${catkin_LIBRARIES} yaml-cpp)


add_executable(joint_state_gui ui_main.cpp)
target_link_libraries(joint_state_gui joint_bar_ui)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Mark executables and/or libraries for installation
install(TARGETS joint_state_gui
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

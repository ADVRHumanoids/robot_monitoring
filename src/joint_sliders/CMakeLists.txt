set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5UiTools  REQUIRED)
find_package(Qt5Widgets  REQUIRED)
find_package(Qt5Quick   REQUIRED)
find_package(xbot2_interface REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(xbot_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 

include_directories(../../include/)

add_definitions(-DXBOT_MSGS_SUPPORT)

## Compile slider widget library
add_library(sliders_widget STATIC
    sliders_widget_mainview.cpp
    sliders_widget.cpp
    sliders_widget_resources.qrc
    )
    
target_link_libraries(sliders_widget PUBLIC 
                        Qt5::Widgets 
                        Qt5::UiTools
                        ${xbot_msgs_TARGETS}
                        ${sensor_msgs_TARGETS}
                        rclcpp::rclcpp
                        xbot2_interface::xbot2_interface)


## Compile main
add_executable(joint_state_sliders sliders_widget_main.cpp)
                                
target_link_libraries(joint_state_sliders PRIVATE 
                        sliders_widget
                        uuid)

set_target_properties(joint_state_sliders PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS joint_state_sliders
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Compile Sliders RViz Panel
#add_library(sliders_rviz_panel SHARED
#    sliders_rviz_panel.cpp
#)

#target_link_libraries(sliders_rviz_panel PRIVATE sliders_widget)

#install(TARGETS sliders_rviz_panel
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES
#  sliders_rviz_panel_plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

### Compile Sliders RQT Panel
#add_library(sliders_rqt SHARED
#    sliders_rqt.cpp
#)

#target_link_libraries(sliders_rqt PRIVATE sliders_widget)

#install(TARGETS sliders_rqt
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES
#  sliders_rqt_plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


add_library(xbot2_gui_monitoring STATIC)

qt_add_qml_module(xbot2_gui_monitoring
    URI Monitoring
    VERSION 1.0
    QML_FILES
    JointCommand.js
    JointCommandCard.qml
    JointDevice.qml
    Monitoring.js
    Monitoring.qml
    RobotModelViewer.js
    RobotModelViewer.qml
    QML_FILES Parameters.qml
    QML_FILES Parameters.js
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_monitoringplugin)

add_subdirectory(BarPlot)

add_subdirectory(SingleJointState)

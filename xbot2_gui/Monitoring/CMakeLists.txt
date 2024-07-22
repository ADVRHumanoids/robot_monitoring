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
    Parameters.qml
    Parameters.js
    SOURCES
    Parameter/parameter_tree_item.h
    Parameter/parameter_tree_item.cpp
    Parameter/parameter_tree_model.h
    Parameter/parameter_tree_model.cpp
    QML_FILES Parameter/BoolDelegate.qml
    QML_FILES Parameter/RealDelegate.qml
    QML_FILES Parameter/VectorDelegate.qml
    QML_FILES Parameter/IntDelegate.qml
    QML_FILES Parameter/DiscreteValuesDelegate.qml
    QML_FILES Parameter/ParameterView.qml
)

target_include_directories(xbot2_gui_monitoring PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Parameter)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_monitoringplugin)

add_subdirectory(BarPlot)

add_subdirectory(SingleJointState)

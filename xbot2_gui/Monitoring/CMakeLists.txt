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
    Parameters.qml
    Parameters.js
    Parameter/BoolDelegate.qml
    Parameter/RealDelegate.qml
    Parameter/VectorDelegate.qml
    Parameter/IntDelegate.qml
    Parameter/DiscreteValuesDelegate.qml
    Parameter/ParameterView.qml
    MonWidget.qml
    MonWidget.js
    SOURCES
    Parameter/parameter_tree_item.h
    Parameter/parameter_tree_item.cpp
    Parameter/parameter_tree_model.h
    Parameter/parameter_tree_model.cpp
    QML_FILES SoftSafetyButton.qml
)

target_include_directories(xbot2_gui_monitoring PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Parameter)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_monitoringplugin)

add_subdirectory(BarPlot)

add_subdirectory(SingleJointState)

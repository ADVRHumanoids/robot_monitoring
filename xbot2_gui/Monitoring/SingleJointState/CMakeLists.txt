add_library(xbot2_gui_monitoring_single_joint STATIC)

qt_add_qml_module(xbot2_gui_monitoring_single_joint
    URI Monitoring.SingleJointState
    VERSION 1.0
    QML_FILES
    AuxSelector.qml
    FieldValue.qml
    PlotButton.qml
    Separator.qml
    SingleJointState.qml
    SingleJointStateForm.ui.qml
    SingleJointStateStack.qml
    SingleJointStateStackForm.ui.qml
    data.js
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_monitoring_single_jointplugin)

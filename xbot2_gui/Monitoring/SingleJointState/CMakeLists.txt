add_library(single_joint STATIC)

qt_add_qml_module(single_joint
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
    SingleJointState1.qml
)

target_link_libraries(xbot2_gui PRIVATE single_jointplugin)

qt_add_library(xbot2_gui_testthings STATIC)

qt_add_qml_module(xbot2_gui_testthings
    URI TestThings
    VERSION 1.0
    QML_FILES
    Playground.qml
    Playground2.qml
    Linfa.qml
    AlberoBuilder.qml
    MotionTab.qml
    MotionTabPlot.qml
    MotionTab.js
    ConfigureMotorPopup.qml
    AnimatedRectangle.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_testthingsplugin)

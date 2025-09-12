add_library(xbot2_gui_joy STATIC)

qt_add_qml_module(xbot2_gui_joy
    URI Joy
    VERSION 1.0
    QML_FILES
    Joy.js
    Joy.qml
    JoyCartesianCard.qml
    Pad.qml
    DualJoy.qml
    QML_FILES GamepadTest.qml
    QML_FILES GamepadInterface.qml
    QML_FILES DummyGamepad.qml
    QML_FILES GamepadManager.qml
    QML_FILES GamepadManagerDummy.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_joyplugin)

add_library(xbot2_gui_concert STATIC)

qt_add_qml_module(xbot2_gui_concert
    URI Concert
    VERSION 1.0
    QML_FILES
    Drilling.qml
    Drilling.js
    DrillingControl.qml
    QML_FILES MotionTab.qml
    QML_FILES DrillTab.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_concertplugin)

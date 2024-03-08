add_library(xbot2_gui_horizon STATIC)

qt_add_qml_module(xbot2_gui_horizon
    URI Horizon
    VERSION 1.0
    QML_FILES
    Horizon.qml
    Horizon.js
    HorizonControl.qml
    PhaseViz.qml
    HorizonViz.qml
    HorizonViz.js
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_horizonplugin)

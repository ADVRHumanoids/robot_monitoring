add_library(xbot2_gui_ecat STATIC)

qt_add_qml_module(xbot2_gui_ecat
    URI Ecat
    VERSION 1.0
    QML_FILES
    Ecat.qml
    Ecat.js
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_ecatplugin)

add_library(xbot2_gui_menu STATIC)

qt_add_qml_module(xbot2_gui_menu
    URI Menu
    VERSION 1.0
    QML_FILES
    Hamburger.qml
    NavBar.qml
    NavDrawer.qml
    NavIcon.qml
    RailMenu.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_menuplugin)

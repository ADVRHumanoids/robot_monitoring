qt_add_library(xbot2_gui_expandable_bottom_bar STATIC)

qt_add_qml_module(xbot2_gui_expandable_bottom_bar
    URI ExpandableBottomBar
    VERSION 1.0
    QML_FILES
    ExpandableBottomBar/NavigationBar.qml
    ExpandableBottomBar/NavigationBarButton.qml
)

qt_add_resources(xbot2_gui_expandable_bottom_bar "xbot2_gui_expandable_bottom_bar_icons"
    PREFIX "ExpandableBottomBar/"
    FILES
    assets/check.png
    assets/home.png
    assets/settings.png
    assets/sun.png
)


target_link_libraries(xbot2_gui PRIVATE xbot2_gui_expandable_bottom_barplugin)

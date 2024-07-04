add_library(xbot2_gui_viewerquick3d STATIC)

qt_add_qml_module(xbot2_gui_viewerquick3d
    URI ViewerQuick3D
    VERSION 1.0
    QML_FILES
    SOURCES
    meshgeometry.h
    meshgeometry.cpp
)

target_link_libraries(xbot2_gui_viewerquick3d PRIVATE Qt6::Quick3D)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_viewerquick3dplugin)

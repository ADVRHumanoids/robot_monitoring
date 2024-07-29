add_library(xbot2_gui_viewerquick3d STATIC)

qt_add_qml_module(xbot2_gui_viewerquick3d
    URI ViewerQuick3D
    VERSION 1.0
    SOURCES
    meshgeometry.h
    meshgeometry.cpp
    pointcloudinstancetable.h
    pointcloudinstancetable.cpp
    QML_FILES
    RobotModelNode.qml
    RobotModelNode.js
    CustomMesh.qml
    Axes3D.qml
    RobotModelViewer.qml
    RobotModelViewer.js
    QML_FILES PointCloud.qml
    QML_FILES MobileJoggingView.qml
)

target_link_libraries(xbot2_gui_viewerquick3d PRIVATE Qt6::Quick3D)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_viewerquick3dplugin)

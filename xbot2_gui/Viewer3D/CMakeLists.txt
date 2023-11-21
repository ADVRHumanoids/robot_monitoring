add_library(xbot2_gui_viewer3d STATIC)

qt_add_qml_module(xbot2_gui_viewer3d
    URI Viewer3D
    VERSION 1.0
    QML_FILES
    CameraController.qml
    CustomOrbitalController.qml
    RobotModelNode.js
    RobotModelNode.qml
    SunLight.qml
    Viewer3D.qml
    VisualEntity.js
    VisualEntity.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_viewer3dplugin)

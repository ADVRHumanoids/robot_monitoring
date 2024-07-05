add_library(xbot2_gui_concert STATIC)

qt_add_qml_module(xbot2_gui_concert
    URI Concert
    VERSION 1.0
    QML_FILES
    Drilling.qml
    Drilling.js
    DrillingControl.qml
    MotionTab.qml
    DrillTab.qml
    Conda.qml
    SliderModelItem.qml
    Sanding.qml
    Sanding.js
    QML_FILES Transportation.qml
    QML_FILES Transportation.js
    QML_FILES TransportationTasks/TeachingTasks.qml
              TransportationTasks/FollowTasks.qml
              TransportationTasks/AutonomousTasks.qml
    QML_FILES TaskBusyIndicator.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_concertplugin)

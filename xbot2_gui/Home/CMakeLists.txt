qt_add_library(xbot2_gui_home STATIC)

qt_add_qml_module(xbot2_gui_home
    URI Home
    VERSION 1.0
    QML_FILES
    HelloScreen.qml
    ServerStatusCard.qml
    QML_FILES Configuration.qml
    QML_FILES ConfigurationGeneral.qml
    QML_FILES ConfigurationAudio.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_homeplugin)

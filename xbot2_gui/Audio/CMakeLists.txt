add_library(xbot2_gui_audio STATIC)

find_package(Qt6 REQUIRED COMPONENTS Multimedia)

qt_add_qml_module(xbot2_gui_audio
    URI Audio
    VERSION 1.0
    QML_FILES
    SOURCES
    QML_FILES Prova.qml
    SOURCES audiosource.h audiosource.cpp
)


target_link_libraries(xbot2_gui PRIVATE xbot2_gui_audioplugin)
target_link_libraries(xbot2_gui_audio PRIVATE Qt6::Multimedia)




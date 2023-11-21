add_library(xbot2_gui_video STATIC)

qt_add_qml_module(xbot2_gui_video
    URI Video
    VERSION 1.0
    QML_FILES
    VideoStream.js
    VideoStream.qml
    VideoStreamCard.qml
    SOURCES
    videostreampainter.h videostreampainter.cpp
)

qt_add_shaders(xbot2_gui_video "xbot2_gui-shaders"
    BATCHABLE # Without this it fails
    PREFIX "/Video/"
    FILES
    yuv2rgb.vert
    yuv2rgb.frag
)

add_subdirectory(theora_cmake)

target_link_libraries(xbot2_gui_video PUBLIC theora::theora Qt6::Gui Qt6::Quick)

target_link_libraries(xbot2_gui_videoplugin PUBLIC theora::theora Qt6::Gui Qt6::Quick)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_videoplugin)




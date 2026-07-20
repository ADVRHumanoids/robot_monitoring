add_library(xbot2_gui_video_mpegts STATIC)

qt_add_qml_module(xbot2_gui_video_mpegts
    URI VideoMpegTs
    VERSION 1.0
    QML_FILES
    VideoStream.js
    VideoStream.qml
    VideoStreamCard.qml
    QML_FILES Example.qml
)

target_link_libraries(xbot2_gui_video_mpegts PUBLIC Qt6::Gui Qt6::Quick)

target_link_libraries(xbot2_gui_video_mpegtsplugin PUBLIC Qt6::Gui Qt6::Quick)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_video_mpegtsplugin)




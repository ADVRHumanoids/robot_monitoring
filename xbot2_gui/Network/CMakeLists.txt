add_library(xbot2_gui_network STATIC)

qt_add_qml_module(xbot2_gui_network
    URI Network
    VERSION 1.0
    QML_FILES
    SOURCES
    udpsocket.h
    udpsocket.cpp
    websocket.h
    websocket.cpp

)

target_link_libraries(xbot2_gui_network PRIVATE Qt6::WebSockets)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_networkplugin)

find_package(Qt6 COMPONENTS
    Protobuf
    ProtobufQuick
)

add_library(xbot2_gui_proto STATIC)

qt_add_qml_module(xbot2_gui_proto
    URI Protobuf
    VERSION 1.0
    QML_FILES
    SOURCES
    protobuf_deserialization.h
    protobuf_deserialization.cpp

)

qt_add_protobuf(xbot2_gui_msgs
    PROTO_FILES
    ${CMAKE_SOURCE_DIR}/proto/generic.proto
    ${CMAKE_SOURCE_DIR}/proto/jointstate.proto
    ${CMAKE_SOURCE_DIR}/proto/text.proto
    ${CMAKE_SOURCE_DIR}/proto/process_output.proto
    ${CMAKE_SOURCE_DIR}/proto/theora_packet.proto
    QML
    QML_URI "xbot2_gui.msgs"
)



target_link_libraries(xbot2_gui_proto PRIVATE Qt6::Protobuf Qt6::ProtobufQuick xbot2_gui_msgs)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_protoplugin xbot2_gui_msgs)



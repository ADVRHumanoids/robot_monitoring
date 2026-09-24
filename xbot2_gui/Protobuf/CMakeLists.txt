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
    ${CMAKE_SOURCE_DIR}/proto/mpeg_ts_datagram.proto
    ${CMAKE_SOURCE_DIR}/proto/pointcloud.proto
    QML
    QML_URI "xbot2_gui.msgs"
)

# qt_add_protobuf creates a shared library on desktop platforms. Install it
# next to xbot2_gui so the Windows loader can resolve it in the staged bundle.
if(NOT ANDROID AND NOT EMSCRIPTEN)
    install(TARGETS xbot2_gui_msgs
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()



target_link_libraries(xbot2_gui_proto PRIVATE Qt6::Protobuf Qt6::ProtobufQuick xbot2_gui_msgs)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_protoplugin xbot2_gui_msgs)


find_package(Qt6 COMPONENTS
    Protobuf
)

set(PROTO_FILES
    joint_states.proto
    tlv.proto
)

qt_add_protobuf(protobuf_messages
    PROTO_FILES
    ${PROTO_FILES}
    QML
    QML_URI xbot2_gui.msgs
)

add_custom_command(
    TARGET protobuf_messages
    COMMAND protoc --python_out=${CMAKE_SOURCE_DIR}/server/proto ${PROTO_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating py from proto"
)

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


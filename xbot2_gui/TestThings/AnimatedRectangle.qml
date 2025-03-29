import QtQuick

Rectangle {

    id: root

    Behavior on height {
        NumberAnimation {
            duration: 333
            easing.type: Easing.OutQuad
        }
    }

    MouseArea {
        anchors.fill: parent
        property real scale: 2.0
        onClicked: {
            root.height *= scale
            scale = 1./scale
        }
    }

}

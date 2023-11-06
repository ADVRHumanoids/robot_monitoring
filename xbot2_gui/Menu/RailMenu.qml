import QtQuick

Item {

    id: root

    property real position: 0.0
    property int railWidth: 80
    property alias menuWidth: menu.width
    property alias color: menu.color
    default property alias content: menu.children


    function open() {
        position = 1
    }

    function close() {
        position = 0
    }

    Behavior on position {
        NumberAnimation {
            duration: 500
            easing.type: Easing.OutQuad
        }
    }

    Rectangle {

        id: menu

        x: (-width + root.railWidth)*(1 - root.position)
        width: 200
        height: parent.height

    }

    Rectangle {
        id: overlayRect
        opacity: position*0.8
        color: Qt.rgba(0, 0, 0, 1)
        anchors {
            left: menu.right
        }
        width: root.position > 0 ? root.width - menu.width - menu.x : 0
        height: menu.height

        MouseArea {
            anchors.fill: parent
            onClicked: {
                root.close()
            }
        }
    }
}

import QtQuick

Item {

    id: root

    property real position: 0.0
    property int railWidth: 80
    property int overlayWidth: 0
    property alias menuWidth: menu.width
    property alias color: menu.color

    implicitWidth: railWidth

    function open() {
        position = 1
    }

    function close() {
        position = 0
    }

    Behavior on position {
        NumberAnimation {
            duration: 500
        }
    }

    Rectangle {

        id: menu

        x: (-width + railWidth)*(1 - position)
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
        height: menu.height
        width : position == 0 ? 0 : root.overlayWidth

        MouseArea {
            anchors.fill: parent
            onClicked: {
                root.close()
            }
        }
    }

    onChildrenChanged: {
        let child = children[children.length - 1]
        child.parent = menu
    }
}

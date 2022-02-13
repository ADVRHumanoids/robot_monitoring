import QtQuick 2.0
import QtQuick.Controls


Item {

    id: root

    property bool menuOpen: false
    property int handleWidth: 50
    property int maxWidth: 400
    property alias model: listView.model

    signal itemSelected(int index)


    function closeMenu() {
        menu.x = -menu.width
        menuOpen = false
    }

    function openMenu() {
        menu.x = 0
        menuOpen = true
    }

    Rectangle {

        id: menu

        x: menuOpen ? 0 : -width
        width: Math.min(root.width - handleWidth, maxWidth)

        anchors {
            top: parent.top
            bottom: parent.bottom
        }

        color: "lightseagreen"

        ScrollView {

            id: scroll
            anchors.fill: parent
            contentWidth: availableWidth
            padding: 10

            ListView {
                id: listView
                width: scroll.contentWidth
                height: contentHeight

                model: listModel
                delegate: listDelegate

                header: MenuHeader {
                    width: listView.width
                    height: 100
                }
            }

            Component {
                id: listDelegate
                MenuEntry {
                    width: listView.width
                    color: menu.color
                    onReleased: {
                        root.itemSelected(index)
                    }
                }
            }
        }

        Behavior on x {
            NumberAnimation {
                duration: 500
                easing.type: Easing.OutQuad
            }
        }


        MouseArea {
            anchors.left: menu.right
            anchors.top: menu.top
            width: menuOpen ? (root.width - menu.width) : handleWidth
            height: menu.height
            drag {
                target: menu
                axis: Drag.XAxis
                minimumX: -menu.width
                maximumX: 0
            }
            onClicked: {
                if(menuOpen) closeMenu()
            }
            onReleased: {
                if( menu.x > -menu.width + 100 ) {
                    openMenu()
                } else {
                   closeMenu()
                }
            }
        }
    }
}

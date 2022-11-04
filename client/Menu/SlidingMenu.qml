import QtQuick 2.0
import QtQuick.Controls
import QtQuick.Controls.Material

Item {

    id: root

    property bool menuOpen: false
    property int handleWidth: 50
    property int maxWidth: 400
    property alias model: listView.model
    property var entryActiveCallback: function (name) { return true; }

    signal itemSelected(int index)

    function evalActiveEntries() {
        for(let i = 0; i < listView.count; i++) {
            let entry = listView.itemAtIndex(i)
            entry.updateEntryActive()
        }
    }

    function closeMenu() {
        menu.x = -menu.width + handleWidth
        menuOpen = false
    }

    function openMenu() {
        evalActiveEntries()
        menu.x = 0
        menuOpen = true
    }

    Rectangle {

        id: menu

        x: menuOpen ? 0 : -width + handleWidth
        width: Math.min(root.width - handleWidth, maxWidth)

        anchors {
            top: parent.top
            bottom: parent.bottom
        }

        color: Material.primary

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

                    onHamburgerClicked: {
                        if(menuOpen) closeMenu()
                        else openMenu()
                    }
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

                    function updateEntryActive() {
                        enabled = entryActiveCallback(index)
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
            width: menuOpen ? (root.width - menu.width) : 5
            height: menu.height
            drag {
                target: menu
                axis: Drag.XAxis
                minimumX: -menu.width + handleWidth
                maximumX: 0
            }
            onClicked: {
                if(menuOpen) closeMenu()
            }
            onReleased: {
                if( menu.x > -menu.width + handleWidth + 100 ) {
                    openMenu()
                } else {
                   closeMenu()
                }
            }
        }
    }
}

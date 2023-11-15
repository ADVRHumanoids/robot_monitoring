import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common
import Menu

Item {

    //
    id: root

    LayoutClassHelper {
        id: layout
        targetWidth: root.width
    }

    default property alias items: container.data

    property list<Item> columnItems

    Item {
        id: container
    }

    Component.onCompleted: {
        // push items that are not repeaters to columnItems
        let tmp = []
        for(let c of items) {
            if(c instanceof Repeater) {
                continue
            }
            tmp.push(c)
        }
        columnItems = tmp

    }

    RowLayout {

        id: row
        anchors.fill: visible ? parent : undefined
        visible: layout.expanded

        Repeater {

            model: root.columnItems.length

            Item {

                id: rowItem
                required property int index

                implicitHeight: rowPane.implicitHeight
                implicitWidth: rowPane.implicitWidth

                Layout.fillWidth:  true
                Layout.fillHeight: true

                Pane {

                    id: rowPane
                    anchors.fill: parent

                    LayoutItemProxy {

                        target: root.columnItems[rowItem.index]
                        width: rowPane.availableWidth
                        height: rowPane.availableHeight

                    }

                    background: Rectangle {
                        radius: 12
                        color: Qt.lighter(rowPane.palette.window)
                    }

                }

            }

        }

    }


    NavBar {

        id: nav

        anchors {
            top: visible ? parent.top : undefined
            left: visible ? parent.left : undefined
            right: visible ? parent.right : undefined
        }

        visible: !row.visible

        Repeater {

            model: root.columnItems.length

            NavButton {
                required property int index
                iconChar: root.columnItems[index].iconChar === undefined ? '\ue574' :  root.columnItems[index].iconChar
                text: root.columnItems[index].iconText
                checkedDisplayMode: AbstractButton.TextUnderIcon
                uncheckedDisplayMode: AbstractButton.TextUnderIcon
                checked: nav.currentIndex === index
//                onClicked: nav.currentIndex = index
            }
        }

        onCurrentIndexChanged: {
            swipe.setCurrentIndex(nav.currentIndex)
        }

    }

    SwipeView {

        id: swipe

        visible: nav.visible

        anchors {
            topMargin: 6
            top: visible ? nav.bottom : undefined
            left: visible ? parent.left : undefined
            right: visible ? parent.right : undefined
            bottom: visible ? parent.bottom : undefined
        }

        onCurrentIndexChanged: {
            nav.setCurrentIndex(swipe.currentIndex)
        }

        Repeater {

            model: root.columnItems.length

            LayoutItemProxy {

                required property int index

                target: root.columnItems[index]
                width: swipe.availableWidth
                height: swipe.availableHeight


            }

        }

    }

}

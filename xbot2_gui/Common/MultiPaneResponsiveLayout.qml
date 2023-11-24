import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common
import Menu

Item {

    signal beforeLayoutChange()
    signal afterLayoutChange()

    //
    id: root

    LayoutClassHelper {
        id: layout
        targetWidth: root.width

        onBeforeLayoutChange: root.beforeLayoutChange()
        onAfterLayoutChange: root.afterLayoutChange()
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
            if(!c.visible) {
                continue
            }
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
        spacing: CommonProperties.geom.margins

        Repeater {

            model: root.columnItems.length

            Item {

                id: rowItem
                required property int index

                implicitHeight: rowPane.implicitHeight
                implicitWidth: rowPane.implicitWidth

                Layout.fillWidth:  true
                Layout.fillHeight: true
                Layout.preferredWidth: proxy.target.columnSize === undefined ?
                                           1 : proxy.target.columnSize
//                Layout.minimumWidth: proxy.Layout.minimumWidth



                Pane {

                    id: rowPane
                    anchors.fill: parent

                    LayoutItemProxy {
                        id: proxy
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


    NavRailWrapper {

        id: nav

        orientation: Qt.Horizontal

        anchors {
            top: visible ? parent.top : undefined
            left: visible ? parent.left : undefined
            right: visible ? parent.right : undefined
        }

        visible: !row.visible && root.columnItems.length > 1

        model: modelItem

        Item {

            id: modelItem

            Repeater {

                model: root.columnItems.length

                Item {
                    required property int index
                    property string name: root.columnItems[index].iconText
                    property string iconText: root.columnItems[index].iconChar
                }
            }
        }

        onCurrentIndexChanged: {
            swipe.setCurrentIndex(nav.currentIndex)
        }

    }


    SwipeView {

        id: swipe

        visible: !row.visible

        clip: true

        anchors {
            topMargin: 6
            top: visible ? (nav.visible ? nav.bottom : parent.top) : undefined
            left: visible ? parent.left : undefined
            right: visible ? parent.right : undefined
            bottom: visible ? parent.bottom : undefined
        }

        onCurrentIndexChanged: {
            nav.currentIndex = swipe.currentIndex
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

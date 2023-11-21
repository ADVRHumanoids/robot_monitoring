import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main
import ExpandableBottomBar
import Font
import Menu
import Joy


ScrollView {

    id: scroll
    contentHeight: mcl.height

    MultiColumnLayout {
        id: mcl
        width: scroll.availableWidth
        columns: 3

        Repeater {

            model: 10

            Rectangle {

                required property int index

                property int columnSpan: index === 5 ? 3 : 1

                height: 100

                border {
                    color: 'red'
                    width: 2
                }

                Behavior on height {
                    NumberAnimation{}
                }

                TapHandler {
                    onTapped: {
                        parent.height += 50
                    }
                }

                Label {
                    anchors.centerIn: parent
                    text: parent.index
                    color: 'red'
                }

            }

        }




    }



}


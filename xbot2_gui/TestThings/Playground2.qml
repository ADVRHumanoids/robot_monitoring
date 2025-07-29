import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Main
import Common


Item {

    id: root
    property ClientEndpoint client
    property int nsplit: 1

    RecursiveSplitView {
        id: split
        anchors.fill: parent

        delegate: Rectangle {
            signal splitVertical()
            signal splitHorizontal()
            signal closeSplit()
            color: Qt.hsva(Math.random(), .8, .8, .8)
            GridLayout {
                anchors.centerIn: parent
                rows: parent.width > parent.height ? 1 : 3
                columns: parent.width > parent.height ? 3 : 1
                TabButton {
                    text: 'Split Vertical'
                    onClicked: splitVertical()
                }
                TabButton {
                    text: 'Split Horizontal'
                    onClicked: splitHorizontal()
                }
                TabButton {
                    text: 'Close Split'
                    onClicked: closeSplit()
                }
            }
            Component.onDestruction: {

                console.log('im dying...')
            }
        }
    }

}


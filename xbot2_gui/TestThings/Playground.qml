import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main

Item {

    id: root
    property ClientEndpoint client

    Card1 {
        id: card
        anchors.centerIn: parent
        width: 400
        name: 'Prova Prova'
        frontItem: Column {
            id: col
            spacing: 10
            Repeater {
                Rectangle {
                    required property int index
                    height: 100
                    width: card.availableContentWidth
                    color: 'red'
                    Text {
                        text: parent.index
                        anchors.centerIn: parent
                    }
                }
                model: 10
            }
        }
    }


//    DebugRectangle {
//        name: 'acci'
//        target: card
//    }

}


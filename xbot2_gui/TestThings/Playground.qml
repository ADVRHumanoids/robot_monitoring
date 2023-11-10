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
//        width: 400
        name: 'Prova Prova'
        frontItem: Rectangle {
            height: 200
            width: 200
        }
    }

    DebugRectangle {
        target: card
    }

}


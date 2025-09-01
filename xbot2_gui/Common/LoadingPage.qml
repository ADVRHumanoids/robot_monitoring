import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Page {

    Column {

        anchors.centerIn: parent
        spacing: 16

        BusyIndicator {
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Label {
            text: 'Loading'
            font.pixelSize: CommonProperties.font.h2
            anchors.horizontalCenter: parent.horizontalCenter
        }

    }

    Behavior on opacity {
        NumberAnimation {
            duration: 333
            easing.type: Easing.OutBack
        }
    }

    visible: opacity > 0

}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common

RowLayout {

    property alias text: title.text
    property alias font: title.font
    signal clicked()

    //
    id: root

    Label {
        id: title
        text: 'Title'
        font.pixelSize: CommonProperties.font.h1
        Layout.fillWidth: true
        wrapMode: Text.Wrap

        MouseArea {
            anchors.fill: parent
            onClicked: root.clicked()
        }
    }
}

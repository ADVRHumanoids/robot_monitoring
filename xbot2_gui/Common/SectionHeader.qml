import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common
import Font

RowLayout {

    property alias text: title.text
    property alias font: title.font
    signal clicked()
    property alias iconText: iconLabel.text

    //
    id: root
    spacing: 16

    Label {

        id: iconLabel

        font.family: CommonProperties.fontAwesome.solid.family
        font.pixelSize: CommonProperties.font.h1

        MouseArea {
            anchors.fill: parent
            onClicked: root.clicked()
        }

    }

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

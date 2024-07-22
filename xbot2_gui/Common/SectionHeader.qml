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
    property int pixelSize: CommonProperties.font.h1

    //
    id: root
    spacing: 16

    Label {

        id: iconLabel

        font.family: CommonProperties.fontAwesome.solid.family
        font.pixelSize: root.pixelSize
        visible: text !== ''

        MouseArea {
            anchors.fill: parent
            onClicked: root.clicked()
        }

    }

    Label {
        id: title
        text: 'Title'
        font.pixelSize: root.pixelSize
        Layout.fillWidth: true
        wrapMode: Text.Wrap

        MouseArea {
            anchors.fill: parent
            onClicked: root.clicked()
        }
    }
}

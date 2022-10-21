import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

Rectangle {

    property int side: 200

    id: outer
    width: side
    height: side
    radius: side/2.
    color: Material.accent

    Rectangle {

        id: inner
        anchors.centerIn: parent
        width: outer.side/4.
        height: width
        radius: width/2.
        color: Qt.darker(outer.color)
        z: 1

        MouseArea {
            anchors.fill: parent
            hoverEnabled: true
        }

    }

    MouseArea {

        anchors.fill: parent
    }

}

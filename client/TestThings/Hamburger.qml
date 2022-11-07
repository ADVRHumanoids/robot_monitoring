import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material

Item {
    id: root
    property int margin: 5
    property int lineWidth: 2
    property color color: Material.primaryTextColor

    signal clicked()

    Rectangle {
        id: mainRect
        color: Qt.rgba(0,0,0,0)
        anchors.fill: parent
        anchors.margins: root.margin

        Rectangle {
            id: centerLine
            height: root.lineWidth
            anchors.centerIn: parent
            width: parent.width
            color: root.color

            transform: Rotation {

            }
        }

        Rectangle {
            id: topLine
            height: root.lineWidth
            anchors.centerIn: parent
            width: parent.width
            color: root.color

            anchors.verticalCenterOffset: mainRect.height / 2

        }

        Rectangle {
            id: bottomLine
            height: root.lineWidth
            anchors.centerIn: parent
            width: parent.width
            color: root.color

            anchors.verticalCenterOffset: -mainRect.height / 2

        }
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true
        onReleased: {
            root.clicked()
        }
    }

}

import QtQuick 2.0
import QtQuick.Controls.Material

Text {
    text: "XBot2 GUI"
    color: "white"
    font.pixelSize: Qt.application.font.pixelSize * 3
    verticalAlignment: Text.AlignVCenter

    signal hamburgerClicked()

    Rectangle {

        property real alpha: mouse.containsMouse ? 0.2 : 0.0

        color: Qt.rgba(0, 0, 0, alpha)
        height: 50
        width: 50

        anchors {
            right: parent.right
            top: parent.top
            margins: 10
        }

        property int margin: 13

        Rectangle {
            color: Material.primaryTextColor
            height: 2
            width: parent.width * 0.8
            x: parent.width * 0.1
            y: parent.margin - height/2
        }

        Rectangle {
            color: Material.primaryTextColor
            height: 2
            width: parent.width * 0.8
            x: parent.width * 0.1
            y: parent.margin + (parent.height - 2*parent.margin) / 2. - height/2

        }

        Rectangle {
            color: Material.primaryTextColor
            height: 2
            width: parent.width * 0.8
            x: parent.width * 0.1
            y: parent.height - parent.margin - height/2
        }

        MouseArea {
            id: mouse
            anchors.fill: parent
            hoverEnabled: true

            onClicked: {
                hamburgerClicked()
            }
        }

        Behavior on alpha {
            NumberAnimation {
                duration: 500
                easing.type: Easing.OutQuad
            }
        }



    }
}

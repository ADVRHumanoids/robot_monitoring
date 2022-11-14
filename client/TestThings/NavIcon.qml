import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material

Item {

    property string name: "A"
    property bool isSelected: false

    signal clicked()

    id: root
    implicitWidth: label.implicitWidth + 8
    implicitHeight: label.implicitHeight + 8

    Rectangle {

        id: background

        color: mouse.containsMouse ? Qt.rgba(0, 0, 0, 0.3) :
                                     isSelected ? Qt.rgba(1, 1, 1, 0.3) :
                                                  Qt.rgba(0, 0, 0, 0)

        anchors {
            fill: parent
        }

        radius: 4
        MouseArea {
            id: mouse
            anchors.fill: parent
            hoverEnabled: true
            onClicked: {
                root.clicked()
            }
        }
    }

    Label {
        id: label
        anchors.centerIn: background
        font.pixelSize: 24
        text: name[0]
        horizontalAlignment: Text.AlignHCenter
    }
}

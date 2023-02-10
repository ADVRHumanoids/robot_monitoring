import QtQuick
import QtQuick.Controls

ProgressBar {

    // public
    property alias text: label.text


    // private
    id: root

    implicitHeight: label.implicitHeight + 6

    Label {
        id: label
        anchors.centerIn: parent
        text: root.value

        Rectangle {
            id: textBackground
            color: "white"
            opacity: 0.2
            radius: 4
            anchors.centerIn: parent
            width: parent.width + 6
            height: parent.height + 6
        }
    }
}

import QtQuick
import QtQuick.Controls

Rectangle {

    property Item target

    anchors.fill: target
    color: Qt.rgba(0, 0, 0, 0)
    border {
        width: 1
        color: 'red'
    }

    Label {
        text: `${parent.width} x ${parent.height} (${parent.implicitWidth} x ${parent.implicitHeight})`
        anchors.centerIn: parent
        onTextChanged: console.log(text)
        color: parent.border.color
        z: 10
    }
}

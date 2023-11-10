import QtQuick
import QtQuick.Controls

Rectangle {

    property Item target
    property string name: 'dbg'

    anchors.fill: target
    color: Qt.rgba(0, 0, 0, 0)
    border {
        width: 1
        color: 'red'
    }

    Label {
        text: `[${name}] ${target.width} x ${target.height} (${target.implicitWidth} x ${target.implicitHeight})`
        anchors.bottom: parent.top
        anchors.margins: 4
        onTextChanged: console.log(text)
        color: parent.border.color
        z: 10
    }
}

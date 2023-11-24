import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Common

Label {


    property bool alert: false

    function setValue(value) {
        _value = value
        root.opacity = 1.0
        timer.restart()
    }

    function setText(text) {
        root.text = text
        root.opacity = 1.0
        timer.restart()
    }

    id: root
    property real _value: 0.0

    text: ' ' + _value.toFixed(2)
    font.bold: alert
    color: alert ? 'red' : palette.text
    opacity: 0.5

    verticalAlignment: Text.AlignVCenter

    Layout.fillWidth: true
    Layout.leftMargin: 5
    Layout.rightMargin: 5
    Layout.minimumWidth: fontMetrics.averageCharacterWidth * 8

    FontMetrics {
        id: fontMetrics
    }

    Timer {
        id: timer
        interval: 500
        onTriggered: {
            alert = false
            root.opacity = 0.5
        }
    }
}

import QtQuick
import xbot2_gui.Common

Rectangle {

    property real min: -1
    property real max: 1
    property real value: 0.5
    property real valueRef: value
    property int barMargin: 0
    property int barRadius: 0
    property alias bar: bar
    property alias refMarker: refMarker

    id: root
    width: 400
    height: textRect.height + 10
    color: "#dddddd"
    radius: barRadius
    border.color: "#ffffff"

    Rectangle {
        id: bar
        anchors.verticalCenter: parent.verticalCenter
        height: parent.height - 2 * barMargin
        color: CommonProperties.colors.primary
        radius: barRadius
    }

    Rectangle {
        id: centerLine
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: value > 0 ? bar.left : bar.right
        height: bar.height
        width: 1
        color: Qt.darker(CommonProperties.colors.primary)
    }

    ReferenceMarker {
        id: refMarker
        height: bar.height
        width: 8
        fillColor: Qt.darker(CommonProperties.colors.primary)
    }

    Rectangle {
        id: textRect
        color: "white"
        opacity: 0.95
        width: valueText.width + 5
        height: valueText.height + 5
        radius: 5
        anchors.centerIn: parent
    }

    Text {
        id: valueText
        text: value.toFixed(2)
        anchors.centerIn: textRect
        color: "black"
    }
}

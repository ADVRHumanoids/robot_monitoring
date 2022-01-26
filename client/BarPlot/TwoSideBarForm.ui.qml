import QtQuick 2.4

Rectangle {

    property real min: -1
    property real max: 1
    property real value: 0.5
    property real valueNormalized: value / (max - min)
    property int barMargin: 0
    property int barRadius: 0
    property alias bar: bar
    property alias centerLine: centerLine

    id: root
    width: 400
    height: textRect.height + 10
    color: "lightGrey"
    radius: barRadius

    Rectangle {
        id: bar
        anchors.verticalCenter: parent.verticalCenter
        height: parent.height - 2 * barMargin
        color: "darkBlue"
        radius: barRadius
    }

    Rectangle {
        id: centerLine
        anchors.verticalCenter: parent.verticalCenter
        height: bar.height
        width: 2
        color: "black"
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

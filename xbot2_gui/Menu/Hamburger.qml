import QtQuick
import QtQuick.Controls

import Common

ToolButton {
    text: CommonProperties.fontAwesome.hamburger
    font.pixelSize: CommonProperties.font.h1
    font.family: CommonProperties.fontAwesome.solid.family
}

//Rectangle {
//    id: root
//    property int margin: 6
//    property int lineWidth: 2
//    property color lineColor: CommonProperties.colors.primaryText
//    color: mouseArea.containsMouse ? Qt.rgba(0, 0, 0, 0.3) : Qt.rgba(0, 0, 0, 0)
//    radius: 4


//    signal clicked()

//    Item {
//        id: mainRect
//        anchors.fill: parent
//        anchors.margins: root.margin

//        Rectangle {
//            id: centerLine
//            height: root.lineWidth
//            y: parent.height/2 - height/2
//            width: parent.width
//            color: root.lineColor


//        }

//        Rectangle {
//            id: topLine
//            height: root.lineWidth
//            y: 0
//            width: parent.width
//            color: root.lineColor
//        }

//        Rectangle {
//            id: bottomLine
//            height: root.lineWidth
//            y: parent.height - height/2
//            width: parent.width
//            color: root.lineColor

//            anchors.verticalCenterOffset: -mainRect.height / 2

//        }
//    }

//    MouseArea {
//        id: mouseArea
//        anchors.fill: parent
//        hoverEnabled: true
//        onReleased: {
//            root.clicked()
//        }
//    }

//}

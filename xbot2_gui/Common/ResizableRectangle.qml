import QtQuick
import QtQuick.Controls

import Common

Item {

    width: 400
    height: 266
    property alias rectX: re.x
    property alias rectY: re.y
    property alias rectWidth: re.width
    property alias rectHeight: re.height
    property alias color: re.color
    property color handleColor: Qt.alpha(CommonProperties.colors.accent, 0.9)
    property alias innerRect: re
    property bool showHandle: true

    onWidthChanged: {
        if(rectX + rectWidth > width) {
            rectX = width - rectWidth
            if(rectX < 0) {
                rectWidth += rectX
                rectX = 0
            }
        }
    }

    onHeightChanged: {
        if(rectY + rectHeight > height) {
            rectY = height - rectHeight
            if(rectY < 0) {
                rectHeight += rectY
                rectY = 0
            }
        }
    }

    signal clicked()

    function center() {
        rectX = width / 2 - rectWidth / 2
        rectY = height / 2 - rectHeight / 2
    }

    function fill() {
        rectX = 0
        rectY = 0
        rectWidth = width
        rectHeight = height
    }

    //
    id: root
    property bool dragging: mouseBottom.drag.active ||
                            mouseTop.drag.active ||
                            mouseLeft.drag.active ||
                            mouseRight.drag.active

    // animate color
    Behavior on color {

        ColorAnimation {

            duration: 222
        }
    }

    Rectangle {
        id: re
        color: 'blue'
        border.color: Qt.lighter(re.color)
        border.width: 2
        width: 100
        height: 100

        // while dragging, anchor to handles
        anchors {
            left: root.dragging ? left.horizontalCenter : undefined
            right: root.dragging ? right.horizontalCenter : undefined
            top: root.dragging ? top.verticalCenter : undefined
            bottom: root.dragging ? bottom.verticalCenter : undefined
        }

        MouseArea {
            anchors.fill: parent
            anchors.margins: 6
            drag.target: parent
            drag.minimumX: 0
            drag.maximumX: root.width - re.width
            drag.minimumY: 0
            drag.maximumY: root.height - re.height

            onDoubleClicked: function(event) {
                re.x = 0
                re.y = 0
                re.width = root.width
                re.height = root.height
                event.accepted = true
            }

            onClicked: root.clicked()
        }
    }


    Rectangle {
        id: top
        height: 20
        width: height
        radius: height/2
        color: root.handleColor
        visible: root.showHandle
        anchors.horizontalCenter: re.horizontalCenter
        anchors.verticalCenter: mouseTop.drag.active ? undefined : re.top

        MouseArea {
            id: mouseTop
            anchors.centerIn: parent
            width: parent.width + 16
            height: parent.height + 16
            drag.target: parent
            drag.axis: Drag.YAxis
            drag.minimumY: -parent.height/2
            drag.maximumY: root.height - parent.height/2
        }
    }

    Rectangle {
        id: bottom
        height: 20
        width: height
        radius: height/2
        color: root.handleColor
        visible: root.showHandle
        anchors.horizontalCenter: re.horizontalCenter
        anchors.verticalCenter: mouseBottom.drag.active || mouseTop.drag.active ? undefined : re.bottom

        MouseArea {
            id: mouseBottom
            anchors.centerIn: parent
            width: parent.width + 16
            height: parent.height + 16
            drag.target: parent
            drag.axis: Drag.YAxis
            drag.minimumY: -parent.height/2
            drag.maximumY: root.height - parent.height/2
        }
    }

    Rectangle {
        id: left
        height: 20
        width: height
        radius: height/2
        color: root.handleColor
        visible: root.showHandle
        anchors.verticalCenter: re.verticalCenter
        anchors.horizontalCenter: mouseLeft.drag.active ? undefined : re.left

        MouseArea {
            id: mouseLeft
            anchors.centerIn: parent
            width: parent.width + 16
            height: parent.height + 16
            drag.target: parent
            drag.axis: Drag.XAxis
            drag.minimumX: -parent.width/2
            drag.maximumX: root.width - parent.width/2
        }
    }

    Rectangle {
        id: right
        height: 20
        width: height
        radius: height/2
        color: root.handleColor
        visible: root.showHandle
        anchors.verticalCenter: re.verticalCenter
        anchors.horizontalCenter: mouseRight.drag.active || mouseLeft.drag.active ? undefined : re.right

        MouseArea {
            id: mouseRight
            anchors.centerIn: parent
            width: parent.width + 16
            height: parent.height + 16
            drag.target: parent
            drag.axis: Drag.XAxis
            drag.minimumX: -parent.width/2
            drag.maximumX: root.width - parent.width/2
        }
    }

}

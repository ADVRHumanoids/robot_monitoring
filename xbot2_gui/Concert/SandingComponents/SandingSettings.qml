import QtQuick
import QtQuick.Layouts
import QtQuick3D
import QtQuick.Controls
import QtQuick.Controls.Basic
import Common

import "."
import "../Sanding3D.js" as Logic

Window {
    id: window
    visible: true
    height: 900
    width: 600

    color: "#24273a"
    title: "Setting"
    signal onClose()

    property int panelID: 0
    property real panelWidth: 1
    property real panelHeight: 1

    property var patch: {
        "width": 1.0,
        "height": 1.0,
        "force": 30.0,
        "velocity": 2.0,
        "x": 0.0,
        "y": 0.0,
        "ID": 0.0,
        "index": 0.0,
        "type": ""
    }

    function uploadData() {
        Logic.upload(patch)
    }

    RoundButton {
        id: close
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.topMargin: 5
        anchors.leftMargin: 5

        background: Rectangle {
                implicitWidth: 20
                implicitHeight: 20
                radius: implicitHeight/2
                color: "#ed8796"
            }

        onClicked: {
            window.close()
            window.onClose()
        }
    }

    ColumnLayout {
        spacing: 20
        anchors.bottom: parent.bottom
        // Layout.alignment: Qt.AlignCenter
        anchors.horizontalCenter: parent.horizontalCenter
        // anchors.verticalCenter: parent.verticalCenter
        // Layout.fillHeight: true
        // Layout.fillWidth: true

        // Layout.anchors.bottom: parent.bottom

        Rectangle {
            width: 240; height: 300
            color: "transparent"

            Component {
                id: delegate
                Item {
                    width: 80; height: 80
                    scale: PathView.iconScale
                    opacity: PathView.iconOpacity
                    property real currentValue: initialValue

                    Column {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        Image { anchors.horizontalCenter: setter.horizontalCenter; width: 64; height: 64; source: icon }
                        Text { id: nameText; text: name; font.pointSize: 16; anchors.horizontalCenter: setter.horizontalCenter; color: "#b7bdf8" }

                        SandingSlider {
                            id: setter
                            // Layout.fillWidth: true
                            defaultValue: initialValue
                            minValue: min
                            maxValue: max
                            step: increment
                            onValueSet: function(value) {
                                console.log(value)
                                patch[(name).toLowerCase()] = value
                                currentValue = value
                            }
                        }

                        Text {
                            id: valueText;
                            text: (currentValue).toFixed(1) + " " + units;
                            font.pointSize: 14;
                            anchors.horizontalCenter: setter.horizontalCenter;
                            color: "#b7bdf8"
                        }
                    }
                }
            }

            PathView {
                anchors.fill: parent
                model: SandingParams {}
                delegate: delegate
                path: Path {
                    startX: 120; startY: 200
                    PathAttribute { name: "iconScale"; value: 1.0 }
                    PathAttribute { name: "iconOpacity"; value: 1.0 }
                    PathQuad { x: 120; y: 75; controlX: 260; controlY: 75 }
                    PathAttribute { name: "iconScale"; value: 0.3 }
                    PathAttribute { name: "iconOpacity"; value: 0.5 }
                    PathQuad { x: 120; y: 200; controlX: -20; controlY: 75 }
                }
            }
        }

        SandingSendingButton {
            // anchors.centerIn: parent
            id: sendButton
            // Layout.fillHeight: true
            Layout.alignment: Qt.AlignHBottom | Qt.AlignCenter
            Layout.bottomMargin: 10
            show: true

        }
    }

    Item {
        id: root
        Layout.alignment: Qt.AlignHTop | Qt.AlignCenter

        height: 600
        width: 600

        property bool dragging: mouseTop.drag.active  || mouseBot.drag.active   ||
                                mouseLeft.drag.active || mouseRight.drag.active

        Rectangle {
            id: fullPanel
            anchors.centerIn: parent
            color: "#363a4f"
            radius: 5
            width: panelWidth * 400 // 1 : 400
            height: 400 //  panelHeight * 400


            Text {
                id: h
                anchors.verticalCenter: fullPanel.verticalCenter
                anchors.right: fullPanel.left
                anchors.rightMargin: 20
                text: (selectedPanel.height/fullPanel.height).toFixed(2)
                font.bold: true
                font.pixelSize: 20
                color: "#cdd6f4"
            }

            Text {
                id: w
                anchors.horizontalCenter: fullPanel.horizontalCenter
                anchors.bottom: fullPanel.top
                anchors.bottomMargin: 20
                // text: qsTr("Width: %1").arg(selectedPanel.width/400)
                text: (selectedPanel.width/400).toFixed(2)
                font.bold: true
                font.pixelSize: 20
                color: "#cdd6f4"
            }

        Rectangle {
            id: selectedPanel
            anchors.centerIn: parent.Center
            width: parent.width
            height: parent.height
            radius: 5
            color: "#6e738d"
            border.color: "#c6a0f6"
            border.width: 2

            anchors {
                left: root.dragging ? left.horizontalCenter : undefined
                right: root.dragging ? right.horizontalCenter : undefined
                top: root.dragging ? top.verticalCenter : undefined
                bottom: root.dragging ? bot.verticalCenter : undefined
            }

            onWidthChanged: {
                if(width < 80 ) {width = 80}
                patch.width = (width/400) // .toFixed(2)
            }
            onHeightChanged: { if (height < 80) {height = 80}
                patch.height = (height/400) // .toFixed(2)
            }

            MouseArea {     // drag mouse area
                anchors.fill: parent
                drag{
                    target: parent
                    minimumX: 0
                    maximumX: (parent.parent.width - parent.width)
                    minimumY: 0
                    maximumY: (parent.parent.height - parent.height)
                    // smoothed: true
                }

                onDoubleClicked: {
                    // TODO: resize
                         // destroy component
                }
            }
        }

            property int rulersSize: 15
            Rectangle {
                id: top
                width: 16
                height: 9
                radius: 5
                x: selectedPanel.x/2
                y: 0
                color: "#c6a0f6"
                // anchors.horizontalCenter: selectedPanel.horizontalCenter
                anchors.horizontalCenter: selectedPanel.horizontalCenter
                anchors.verticalCenter: mouseTop.drag.active ? undefined : selectedPanel.top
                // anchors.verticalCenterOffset:  mouseTop.drag.active ? 0 : 2
                // anchors.verticalCenterOffset: 2
                onYChanged: {
                    var localY = top.mapToItem(fullPanel, 0, 0).y
                    if (localY > 316) {y = 316}
                    var distanceFromBot = top.mapFromItem(bot, 0, 0).y
                    if (distanceFromBot < 80) { y = bot.mapToItem(fullPanel, 0, 0).y - 80 }

                    patch.y =  (patch['index']+ 1) - ((y+5)/fullPanel.height) // .toFixed(2)
                }
                MouseArea {
                    id: mouseTop
                    anchors.fill: parent
                    drag.target: parent
                    drag.axis: Drag.YAxis
                    drag.minimumY: -parent.height/2
                    drag.maximumY: fullPanel.height
                }
            }

            Rectangle {
                id: left
                width: 9
                height: 16
                radius: 5
                color: "#c6a0f6"
                anchors.verticalCenter: selectedPanel.verticalCenter
                anchors.horizontalCenter: mouseLeft.drag.active ? undefined : selectedPanel.left
                onXChanged: {
                    var localX = left.mapToItem(fullPanel, 0, 0).x
                    if (localX > 316 ) {x = 316}
                    var distanceFromRight = left.mapFromItem(right, 0, 0).x
                    if (distanceFromRight < 80) { x = right.mapToItem(fullPanel, 0, 0).x -80 }

                    patch.x = ((-(x+5)+fullPanel.width/2)/fullPanel.width ) // .toFixed(2) //+ (fullPanel.width/2))
                }
                MouseArea {
                    id: mouseLeft
                    anchors.fill: parent
                    drag {
                        target: parent
                        axis: Drag.XAxis
                        minimumX: -parent.width/2
                        maximumX: fullPanel.width
                    }

                }
            }

            Rectangle {
                id: bot
                width: 16
                height: 9
                radius: 5
                // x: parent.x / 2
                // y: parent.y
                color: "#c6a0f6"
                anchors.horizontalCenter: selectedPanel.horizontalCenter
                anchors.verticalCenter: mouseBot.drag.active || mouseTop.drag.active ? undefined : selectedPanel.bottom
                onYChanged: {
                    var localY = bot.mapToItem(selectedPanel, 0, 0).y
                    if (localY < 80) {
                       y = 80 + top.mapToItem(fullPanel, 0, 0).y
                    }

                    //from 95.5 to 495.5
                }
                // anchors.
                MouseArea {
                    id: mouseBot
                    anchors.fill: parent
                    drag {
                        target: parent
                        axis: Drag.YAxis
                        minimumY: -parent.height/2 + 80
                        maximumY: fullPanel.height
                    }
                }
            }



            Rectangle {
                id: right
                width: 9
                height: 16
                radius: 5
                color: "#c6a0f6"
                anchors.rightMargin: 5
                anchors.verticalCenter: selectedPanel.verticalCenter
                anchors.horizontalCenter: mouseRight.drag.active || mouseLeft.drag.active ? undefined : selectedPanel.right

                onXChanged: {
                    var distanceFromLeft = right.mapToItem(left, 0, 0).x
                    if (distanceFromLeft < 80) { x = left.mapToItem(fullPanel, 0, 0). x + 80 }
                }

                MouseArea {
                    id: mouseRight
                    anchors.fill: parent
                    drag {
                        target: parent
                        axis: Drag.XAxis
                        minimumX: -parent.width/2
                        maximumX: fullPanel.width
                    }
                }
            }
        }
    }
}



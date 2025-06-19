import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Shapes

import "../Sanding3D.js" as Logic

    Rectangle {

        property color pulsingColor: "#5b6078"

        property bool readyToSand: false
        id: startButton
        visible: readyToSand
        width: 200
        height: 100
        color: pulsingColor
        radius: height / 2
        Layout.alignment: Qt.AlignCenter
        // border.width: 2
        SequentialAnimation on pulsingColor {
            loops: Animation.Infinite
            running: startButton.readyToSand

            ColorAnimation {
                from: pulsingColor
                to: "#81c8be"
                duration: 500
                easing.type: Easing.InOutQuad
            }

            ColorAnimation {
                from: "#81c8be"
                to: pulsingColor
                duration: 500
                easing.type: Easing.InOutQuad
            }
        }
        Text {
            text: qsTr("Start")
            font.bold: false
            font.pixelSize: 27
            font.letterSpacing: 1
            opacity: enabled ? 1.0 : 0.3
            color: "#cad3f5" // scanButton.down ? "#a5adce" : "#c6d0f5"
            horizontalAlignment: parent.AlignHCenter
            verticalAlignment: parent.AlignVCenter
            elide: Text.ElideRight
            anchors.centerIn: parent
        }

        Button {
            id: startButtonInteractive
            anchors.centerIn: parent
            opacity: 0
            onClicked: {
                Logic.startMission()

            }
        }

    }

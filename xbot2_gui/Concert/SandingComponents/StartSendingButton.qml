import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Shapes

import "../Sanding3D.js" as Logic

    Rectangle {
        property bool readyToSand: false
        id: startButton
        visible: readyToSand
        width: 200
        height: 100
        color: "#5b6078"
        radius: height / 2
        Layout.alignment: Qt.AlignCenter
        // border.width: 2

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

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Shapes

import "../Sanding3D.js" as Logic

    Rectangle {
        property bool isReady: true
        property color pulsingColor: "#5b6078"

        id: startToolButton
        visible: isReady
        width: 200
        height: 100
        color: pulsingColor
        radius: height / 2
        Layout.alignment: Qt.AlignCenter

        SequentialAnimation on pulsingColor {
            loops: Animation.Infinite
            running: isReady

            ColorAnimation {
                from: pulsingColor
                to: "#f38ba8"
                duration: 500
                easing.type: Easing.InOutQuad
            }


            ColorAnimation {
                from: "#f38ba8"
                to: pulsingColor
                duration: 500
                easing.type: Easing.InOutQuad
            }
        }

        Text {
            text: qsTr("Start Tool")
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
                Logic.startTool()

            }
        }

    }

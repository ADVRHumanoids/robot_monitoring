import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Shapes

import "../Sanding3D.js" as Logic
GridLayout {
    id: gridScanning
    property bool show: true
    property real angle: 30
    flow: GridLayout.TopToBottom
    uniformCellWidths: true
    rows: 2
    rowSpacing: 10

    Rectangle {
        id: scanButton
        visible: show
        width: 200
        height: 100
        color: "#5b6078"
        radius: height / 2
        Layout.alignment: Qt.AlignCenter
        // border.width: 2

        Text {
            text: qsTr("Scan")
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
            id: scanButtonInteractive
            anchors.centerIn: parent
            opacity: 0
            onClicked: {
                gridScanning.show = false
                Logic.startScanning(control.value)
                scanningLoader.visible = true
                scanningProgress.text = "0%"
                // TODO add progress bar
                console.log("Passing angle: ", control.value)
                console.log("Scanning. . . ")

            }
        }

    }

    SpinBox {
        id: control
        visible: gridScanning.show
        value: gridScanning.angle
        // editable: true
        stepSize: 15
        from: 0
        to: 360

        Layout.alignment: Qt.AlignCenter

        background: Rectangle {
            // implicitWidth: 140
            color: "#5b6078" //"#5b6078"
            radius: width/2
        }
        contentItem: Text {
            z: 2
            text: control.textFromValue(control.value, control.locale) + "°"
            font.bold: true
            font.pixelSize: 15
            color: "#cad3f5"
            // font.bold: true
            // selectionColor: "#21be2b"
            // selectedTextColor: "#ffffff"
            horizontalAlignment: Qt.AlignHCenter
            verticalAlignment: Qt.AlignVCenter

            // readOnly: !control.editable
            // validator: control.validator
            // inputMethodHints: Qt.ImhFormattedNumbersOnly
        }
        /*

        up.indicator: Rectangle {
            x: control.mirrored ? 0 : parent.width - width
            height: parent.height
            implicitWidth: 40
            implicitHeight: 40
            color: "#5b6078" // color: control.up.pressed ? "#e4e4e4" : "#f6f6f6"
            // border.color: enabled ? "#21be2b" : "#bdbebf"
            radius: width/2

            Text {
                text: "x"
                font.pixelSize: control.font.pixelSize
                color: "#cad3f5"
                anchors.fill: parent
                fontSizeMode: Text.Fit
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        down.indicator: Rectangle {
            x: control.mirrored ? parent.width - width : 0
            height: parent.height
            implicitWidth: 40
            implicitHeight: 40
            color: "#5b6078" // control.down.pressed ? "#e4e4e4" : "#f6f6f6"
            // border.color: enabled ? "#21be2b" : "#bdbebf"
            radius: width/2

            Text {
                text: "x"
                font.pixelSize: control.font.pixelSize
                color: "#cad3f5"
                anchors.fill: parent
                fontSizeMode: Text.Fit
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        } */
    }
}



// MouseArea {
//     id: mouseArea
//     anchors.fill: parent
//     onClicked: scanButton.state === 'clicked' ? scanButton.state = "" : scanButton.state = 'clicked';
// }

// states: [
//     State {
//         name: "clicked"
//         PropertyChanges { target: scanButton; visible: false }
//     }
// ]

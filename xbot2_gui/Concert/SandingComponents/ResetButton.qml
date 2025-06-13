import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Shapes

import "../Sanding3D.js" as Logic

Rectangle {
    id: roundedRectangle
    property bool show: true
    visible: show
    width: 200*0.75
    height: 100*0.75
    color: "#f5a97f"
    radius: height / 2
    // border.width: 2-

    Text {
        text: qsTr("Reset")
        font.bold: false
        font.pixelSize: 27
        font.letterSpacing: 1
        opacity: enabled ? 1.0 : 0.3
        color: "#1e2030" // scanButton.down ? "#a5adce" : "#c6d0f5"
        horizontalAlignment: parent.AlignHCenter
        verticalAlignment: parent.AlignVCenter
        elide: Text.ElideRight
        anchors.centerIn: parent
    }

    Button {
        id: resetButton
        anchors.centerIn: parent
        height: parent.height
        width: parent.width

        opacity: 0
        onClicked: {
            console.log("RESET")
            scanningLoader.visible = false
            scanningButton.show = true
            scanningProgress.text = "0%"
            reset()
            removeWall()
            // TODO add progress bar
        }
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

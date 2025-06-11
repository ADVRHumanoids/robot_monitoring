import QtQuick
import QtQuick3D
import QtQuick.Controls
import "."

Rectangle {
    property bool show: false
    id: sandingPopup
    visible: true // show
    width: 100 // 200
    height: 50 // 100
    color: "#8aadf4"
    radius: height / 2
    Text {
        id: txt
        text: qsTr("Send")
        font.bold: false
        font.pixelSize: 18
        font.letterSpacing: 1
        opacity: enabled ? 1.0 : 0.3
        color: "#363a4f"// "#cad3f5" // scanButton.down ? "#a5adce" : "#c6d0f5"
        horizontalAlignment: parent.AlignHCenter
        verticalAlignment: parent.AlignVCenter
        elide: Text.ElideRight
        anchors.centerIn: parent
    }

    state: "ready"
    states : [

        State {
            name: "ready"
            PropertyChanges {target: sandingPopup; color: "#8aadf4"}
        },

        State {
            name: "started"
            PropertyChanges {target: sandingPopup; color:"#a6e3a1" }
            PropertyChanges {target: txt; color: "#363a4f"}
        }

    ]

    Button {
        id: sendButton
        anchors.centerIn: parent
        opacity: 0
        // contentItem: Text {
        //     text: send.text
        //     font.bold: true
        //     font.pixelSize: 14
        //     font.letterSpacing: 1
        //     opacity: enabled ? 1.0 : 0.3
        //     color: send.down ? "#a5adce" : "#c6d0f5"
        //     horizontalAlignment: Text.AlignHCenter
        //     verticalAlignment: Text.AlignVCenter
        //     elide: Text.ElideRight
        // }

        // background: Rectangle {
        //         color: "#8caaee"
        //         radius: 10
        //     }

        onClicked: {
            // parent.visible = false
            // for (var i = 0; i < wall.count; i++) {
            //     if (wall.children[i].isPicked) {
            //         wall.children[i].isPicked = false
            //         wallModel.setState(i)
            //     }
            // }
            // console.log(mouseArea)
            // wall.ready = false
            // // parent.reset()
            // mauseArea.deleteSetting()
            sandingPopup.state = "started"
            uploadData()
        }
    }
}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

Rectangle {

    width: col.implicitWidth
    height: col.implicitHeight
    color: root.status === "Stopped" ? Qt.lighter(Material.backgroundColor) : Material.color(Material.Teal)
    radius: 4

    // aliases to enable usage from the parent component
    property alias startBtn: startBtn
    property alias stopBtn: stopBtn
    property alias killBtn: killBtn
    property alias configureBtn: configureBtn

    Column {
        anchors.centerIn: parent
        id: col
        spacing: 10
        topPadding: 5
        leftPadding: 5
        rightPadding: 5
        bottomPadding: 5

        Label {
            id: statusLabel
            anchors.horizontalCenter: parent.horizontalCenter
            text: root.name
        }

        Row {

            id: btnRow
            anchors.horizontalCenter: parent.horizontalCenter
            spacing: 5

            Item {

                width: startBtn.implicitWidth
                height: startBtn.implicitHeight

                Button {
                    anchors.fill: parent
                    id: startBtn
                    text: "Start"
                    visible: root.status === "Stopped"
                }

                Button {
                    anchors.fill: parent
                    id: stopBtn
                    text: "Stop"
                    z: 0
                    visible: root.status !== "Stopped"
                }

            }

            Button {
                id: killBtn
                text: "Kill"
            }


        }

        Button {
            anchors.horizontalCenter: parent.horizontalCenter
            id: configureBtn
            text: root.configPanelOpen ? "Cancel" : "Configure"
            enabled: root.status === "Stopped"
        }
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/


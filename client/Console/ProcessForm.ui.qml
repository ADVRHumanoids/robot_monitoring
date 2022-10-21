import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11

Column {

    id: mainColumn
    spacing: 10

    // aliases to enable usage from the parent component
    property alias startBtn: startBtn
    property alias stopBtn: stopBtn
    property alias killBtn: killBtn
    property alias configureBtn: configureBtn

    Label {
        id: statusLabel
        anchors.horizontalCenter: parent.horizontalCenter
        text: root.name + ": " + root.status
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

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/


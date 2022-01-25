import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11

Rectangle {

    id: mainRect
    width: 320
    height: 568
    color: "#d8edd5"

    property string serverHost: hostField.text
    property string serverPort: portField.text
    property alias applyBtn: applyBtn
    property alias msgText: msgText
    property alias statusText: statusText
    property alias mainLayout: mainLayout

    ColumnLayout {

        id: mainLayout
        anchors.fill: parent
        anchors.margins: 50

        TextArea {
            id: titleText
            text: "Xbot2 Gui\n"
            horizontalAlignment: Text.AlignHCenter

            font.weight: Font.DemiBold
            font.family: "Arial"
            font.pointSize: 20

            Layout.fillWidth: true
        }

        TextArea {
            id: statusText
            Layout.fillWidth: true
            text: "Trying to reach server at " + serverHost + ":" + serverPort
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
        }

        TextArea {
            id: msgText
            Layout.fillWidth: true
            text: ""
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
            font.pointSize: 12
        }

        Item {
            Layout.fillHeight: true
        }

        GridLayout {
            id: formLayout
            Layout.alignment: Qt.AlignHCenter
            columns: 2

            Label {
                text: "Host"
            }
            TextField {
                id: hostField
                text: appData.hostname
            }

            Label {
                text: "Port"
            }
            TextField {
                id: portField
                text: appData.port
            }
        }

        Item {
            Layout.preferredHeight: 10
        }

        Button {
            id: applyBtn
            text: "Retry"
            Layout.alignment: Qt.AlignHCenter
            focus: true
        }
    }
}

/*##^##
Designer {
    D{i:0;width:360}D{i:2}D{i:3}D{i:4}D{i:5}D{i:6}D{i:8}D{i:9}D{i:10}D{i:11}D{i:7}D{i:12}
D{i:13}D{i:1}
}
##^##*/


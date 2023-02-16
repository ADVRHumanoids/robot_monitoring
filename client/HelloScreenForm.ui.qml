import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "TestThings"

import xbot2_gui.common

MaterialResponsiveGrid {

    id: mainGrid

    property string serverHost: hostField.text
    property string serverPort: portField.text
    property alias applyBtn: applyBtn
    property alias resetUiBtn: resetUiBtn
    property alias msgText: msgText
    property alias statusText: statusText
    property alias mainLayout: mainLayout

    SectionHeader {
        property int columnSpan: mainGrid.columns
        text: 'XBot2 GUI'
    }

    Card {
        name: 'Server status'

        configurable: false

        frontItem: GridLayout {
            id: formLayout
            anchors.fill: parent
            columns: 2
            columnSpacing: 20

            Label {
                text: "Host"
            }
            TextField {
                id: hostField
                text: client.hostname
            }

            Label {
                text: "Port"
            }
            TextField {
                id: portField
                text: client.port
            }

            Label {
                text: "Data RX (KB/s)"
            }
            TextField {
                text: client.port
            }

            Label {
                text: "Data TX (KB/s)"
            }
            TextField {
                text: client.port
            }
        }
    }

    ColumnLayout {

        id: mainLayout
        anchors.fill: parent
        anchors.margins: 50

        TextArea {
            id: titleText
            text: "xbot2 gui\n"
            horizontalAlignment: Text.AlignHCenter

            font.weight: Font.DemiBold
            font.family: "Arial"
            font.pointSize: CommonProperties.font.h1

            Layout.fillWidth: true
        }

        TextArea {
            id: statusText
            Layout.fillWidth: true
            text: "Server at " + serverHost + ":" + serverPort
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
            font.pointSize: CommonProperties.font.h4
        }

        TextArea {
            id: msgText
            Layout.fillWidth: true
            text: ""
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
            font.pointSize: CommonProperties.font.h4
        }

        Item {
            Layout.fillHeight: true
        }

        GridLayout {
            id: formLayout
            Layout.alignment: Qt.AlignHCenter
            columns: 2
            columnSpacing: 20

            Label {
                text: "Host"
            }
            TextField {
                id: hostField
                text: client.hostname
            }

            Label {
                text: "Port"
            }
            TextField {
                id: portField
                text: client.port
            }
        }

        Item {
            Layout.preferredHeight: 10
        }

        Row {

            spacing: 16
            Layout.alignment: Qt.AlignHCenter

            Button {
                id: applyBtn
                text: "Retry"
                focus: true
            }

            Button {
                id: resetUiBtn
                text: "Reset UI"
                focus: true
            }

        }
    }
}

/*##^##
Designer {
    D{i:0;width:360}D{i:2}D{i:3}D{i:4}D{i:5}D{i:6}D{i:8}D{i:9}D{i:10}D{i:11}D{i:7}D{i:12}
D{i:13}D{i:1}
}
##^##*/


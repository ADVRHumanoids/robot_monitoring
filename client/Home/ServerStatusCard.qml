import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import xbot2_gui.Common
import "../Common"
import ".."


Card {

    property ClientEndpoint client
    property alias msgText: msgText
    property alias rxKbps: statsTimer.rxKbps
    property alias txKbps: statsTimer.txKbps
    signal statsUpdated()


    // private
    id: root
    name: 'Server status'
    configurable: false

    function updateServerUrl() {
         client.hostname = hostField.text
         client.port = parseInt(portField.text)
         client.active = true
     }

    Timer {

        id: statsTimer
        interval: 300
        repeat: true
        property int _rxBytes: 0
        property int _txBytes: 0
        property int rxKbps: 0
        property int txKbps: 0

        Component.onCompleted: start()

        onTriggered: {
            let rx = client.bytesRecv - _rxBytes
            let tx = client.bytesSent - _txBytes
            rxKbps = rx/interval*8
            txKbps = tx/interval*8
            _rxBytes = client.bytesRecv;
            _txBytes = client.bytesSent;
            root.statsUpdated()
        }
    }

    frontItem: GridLayout {
        id: formLayout
        anchors.fill: parent
        columns: 2
        columnSpacing: CommonProperties.geom.spacing

        Label {
            Layout.columnSpan: 2
            text: `Connecting to ${client.hostname}:${client.port}`
        }

        Label {
            text: "Host"
        }
        TextField {
            id: hostField
            Layout.fillWidth: true
            text: client.hostname
            onAccepted: {
                root.updateServerUrl()
            }
        }



        Label {
            text: "Port"
        }
        TextField {
            id: portField
            Layout.fillWidth: true
            text: client.port
            onAccepted: {
                root.updateServerUrl()
            }
        }

        Button {
            Layout.alignment: Qt.AlignHCenter
            Layout.columnSpan: 2
            text: 'Apply'
            onClicked: {
                root.updateServerUrl()
            }
        }

        Label {
            text: "Status"
        }
        TextArea {
            id: msgText
            Layout.fillWidth: true
            text: ''
            readOnly: true
            wrapMode: TextEdit.Wrap
        }

        Label {
            text: "Data RX (kbps)"
        }
        TextField {
            id: rxText
            Layout.fillWidth: true
            text: statsTimer.rxKbps.toFixed(1)
            readOnly: true
        }

        Label {
            text: "Data TX (kbps)"
        }
        TextField {
            id: txText
            Layout.fillWidth: true
            text: statsTimer.txKbps.toFixed(1)
            readOnly: true
        }

        Label {
            text: "Ping (ms)"
        }
        TextField {
            id: pingText
            Layout.fillWidth: true
            text: client.srvRtt.toFixed(1)
            readOnly: true
        }
    }
}

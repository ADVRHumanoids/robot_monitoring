import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import Main
import Common


Card1 {

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

    Timer {
        id: delayedConnect
        interval: 1000
        onTriggered: root.updateServerUrl()
    }

    toolButtons: [
        Button {
            text: 'Reset'
            onClicked: {
                client.doRequest('POST', '/restart', '')
                delayedConnect.restart()
            }
        }
    ]

    frontItem: GridLayout {
        id: formLayout
        anchors.fill: parent
        columns: 2
        columnSpacing: CommonProperties.geom.spacing
        rowSpacing: CommonProperties.geom.spacing

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
            text: 'Connect'
            onClicked: {
                root.updateServerUrl()
            }
        }

        Label {
            text: "Server version"
        }
        TextArea {
            id: versionText
            Layout.fillWidth: true
            text: '--'
            readOnly: true
            wrapMode: TextEdit.Wrap
            enabled: client.isConnected
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
            enabled: client.isConnected
        }

        Label {
            text: "Data RX (kbps)"
        }
        TextField {
            id: rxText
            Layout.fillWidth: true
            text: statsTimer.rxKbps.toFixed(1)
            readOnly: true
            enabled: client.isConnected
        }

        Label {
            text: "Data TX (kbps)"
        }
        TextField {
            id: txText
            Layout.fillWidth: true
            text: statsTimer.txKbps.toFixed(1)
            readOnly: true
            enabled: client.isConnected
        }

        Label {
            text: "Ping (ms)"
        }
        TextField {
            id: pingText
            Layout.fillWidth: true
            text: client.srvRtt.toFixed(1)
            readOnly: true
            enabled: client.isConnected
        }
    }

    Connections {
        target: client
        onIsConnectedChanged: {
            if(client.isConnected) {
                client.doRequestAsync('GET', '/version', '')
                .then((res) => {
                          versionText.text = res.version
                      })
                .catch((err) => {})
            }
        }
    }
}

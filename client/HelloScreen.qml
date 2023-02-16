import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import xbot2_gui.common
import "TestThings"

Item {


    property ClientEndpoint client: undefined

    signal restartUi()

    function setError(msg) {
        msgText.text = "Error: " + msg
        msgText.color = CommonProperties.colors.err
    }

    function setConnected(msg) {
        msgText.text = "Status OK: " + msg
        msgText.color = CommonProperties.colors.ok
    }

    function setProgress(msg) {
        statusText.text = msg
    }


    // private
    id: root

    MaterialResponsiveGrid {

        id: mainGrid

        anchors.fill: parent

        SectionHeader {
            property int columnSpan: mainGrid.columns
            text: 'XBot2 GUI'
        }

        Card {
            name: 'Server status'

            configurable: false

            Timer {
                interval: 1000
                repeat: true
                property int _rxBytes: 0
                property int _txBytes: 0

                Component.onCompleted: start()

                onTriggered: {
                    let rx = client.bytesRecv - _rxBytes
                    let tx = client.bytesSent - _txBytes
                    rxText.text = (rx/1000.0*8).toFixed(1)
                    txText.text = (tx/1000.0*8).toFixed(1)
                    _rxBytes = client.bytesRecv;
                    _txBytes = client.bytesSent;
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
                    text: '0'
                    readOnly: true
                }

                Label {
                    text: "Data TX (kbps)"
                }
                TextField {
                    id: txText
                    Layout.fillWidth: true
                    text: '0'
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
    }

   function updateServerUrl() {
        client.hostname = hostField.text
        client.port = parseInt(portField.text)
        client.active = true
    }
}

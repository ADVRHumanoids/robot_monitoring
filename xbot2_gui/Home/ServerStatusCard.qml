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
        interval: 2000
        repeat: true

        property int _rxBytes: 0
        property int _txBytes: 0
        property int _tmNum: 0
        property int _tmDroppedNum: 0

        property int rxKbps: 0
        property int txKbps: 0
        property int tmHz: 0
        property int tmDroppedHz: 0

        property var lastStats: Object()
        property int numProcStats: 0

        function initProcStats() {
            lastStats['cpu_usage_main'] = 0
            lastStats['cpu_usage_total'] = 0
            lastStats['memory_usage_MB'] = 0
            numProcStats = 0
        }

        Component.onCompleted: {
            initProcStats()
            start()
        }

        onTriggered: {

            let rx = client.bytesRecv - _rxBytes
            rxKbps = rx/interval*8
            _rxBytes = client.bytesRecv;

            let tx = client.bytesSent - _txBytes
            txKbps = tx/interval*8
            _txBytes = client.bytesSent;

            let tm = client.jsMsgRecv - _tmNum
            tmHz = tm / interval * 1000.
            _tmNum = client.jsMsgRecv

            let tmDropped = client.jsDropped - _tmDroppedNum
            tmDroppedHz = tmDropped / interval * 1000.
            _tmDroppedNum = client.jsDropped

            cpumemText.mainCpuPerc = lastStats.cpu_usage_main / numProcStats
            cpumemText.totalCpuPerc = lastStats.cpu_usage_total / numProcStats
            cpumemText.memUsageMB = lastStats.memory_usage_MB / numProcStats
            initProcStats()

            root.statsUpdated()
        }
    }

    Timer {
        id: delayedConnect
        interval: 3000
        onTriggered: root.updateServerUrl()
    }

    toolButtons: [
        Button {
            text: 'Restart server'
            onClicked: {
                client.doRequest('POST', '/restart', '')
                delayedConnect.restart()
            }
        }
    ]

    frontItem: ColumnLayout {

        anchors.fill: parent
        spacing: CommonProperties.geom.spacing * 1.33

        GridLayout {

            Layout.fillWidth: true

            id: formLayout
            columns: 2
            columnSpacing: CommonProperties.geom.spacing * 1.33
            rowSpacing: CommonProperties.geom.spacing * 1.33

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
                placeholderText: text === '' ? 'Enter server host' : ''
            }



            Label {
                text: "Port"
            }
            TextField {
                id: portField
                Layout.fillWidth: true
                text: client.port > 0 ? client.port : ''
                onAccepted: {
                    root.updateServerUrl()
                }
                placeholderText: text === '' ? 'Enter server port' : ''
            }

            Button {
                Layout.alignment: Qt.AlignHCenter
                Layout.columnSpan: 2
                text: 'Connect'
                onClicked: {
                    root.updateServerUrl()
                }
            }
        }

        GridLayout {

            Layout.fillWidth: true

            columns: Math.ceil(width / 300)
            columnSpacing: CommonProperties.geom.spacing * 1.33
            rowSpacing: CommonProperties.geom.spacing * 1.33
            uniformCellWidths: true

            FramedValue {
                id: msgText
                title: 'Connection status'
                value1: '--'
                enabled: client.isConnected
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            FramedValue {
                id: versionText
                title: 'Server version'
                value1: '--'
                enabled: client.isConnected
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            FramedValue {
                id: pingText
                title: 'Ping'
                value1: `${client.srvRtt.toFixed(1)} ms`
                enabled: client.isConnected
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            FramedValue {
                id: dataText
                title: 'Data rate [kbps]'
                value1: `${statsTimer.rxKbps.toFixed(1)} kbps RX`
                value2: `${statsTimer.txKbps.toFixed(1)} kbps TX`
                enabled: client.isConnected
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            FramedValue {
                id: teleText
                title: 'Telemetry rate [Hz]'
                value1: `${statsTimer.tmHz.toFixed(0)} received`
                value2: `${statsTimer.tmDroppedHz.toFixed(0)} dropped`
                enabled: client.isConnected
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            FramedValue {
                property real mainCpuPerc: 0
                property real totalCpuPerc: 0
                property int memUsageMB: 0
                id: cpumemText
                title: 'Server resource usage'
                value1: `CPU: ${totalCpuPerc.toFixed(1)}%  (main: ${mainCpuPerc.toFixed(1)}%)`
                value2: `RAM: ${memUsageMB} MB`
                enabled: client.isConnected
                Layout.fillWidth: true
                Layout.fillHeight: true
            }
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
        onObjectReceived: function(obj) {
            if(obj.type !== 'server_stats') return
            for(let k of Object.keys(statsTimer.lastStats)) {
                statsTimer.lastStats[k] += obj[k]
            }
            statsTimer.numProcStats += 1
        }
    }
}

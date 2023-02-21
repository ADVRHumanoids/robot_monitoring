import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import xbot2_gui.common
import "../Common"
import "../LivePlot"
import ".."

Item {


    property ClientEndpoint client: undefined

    signal restartUi()

    function setError(msg) {
        srvStatus.msgText.text = "Error: " + msg
        srvStatus.msgText.color = CommonProperties.colors.err
    }

    function setConnected(msg) {
        srvStatus.msgText.text = "Status OK: " + msg
        srvStatus.msgText.color = CommonProperties.colors.ok
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

        ServerStatusCard {
            id: srvStatus
            client: root.client
            onStatsUpdated: {
                let t = appData.getTimeNs()*1e-9 - statsPlot.t0
                statsPlot.addPoint(statsPlot.currSeries['rx data'],
                                   t, rxKbps)
                statsPlot.addPoint(statsPlot.currSeries['tx data'],
                                   t, txKbps)
                statsPlot.addPoint(statsPlot.currSeries['server rtt'],
                                   t, client.srvRtt)
            }
        }

        Card {

            height: srvStatus.height

            property int columnSpan: 8

            frontItem: Plotter {
                id: statsPlot
                anchors.fill: parent
                property real t0: appData.getTimeNs()*1e-9
                axisLeftTitle: 'RTT [ms]'
                axisRightTitle: 'Data rate [kbps]'

            }

            Component.onCompleted: {
                statsPlot.addSeries('server rtt', {})
                statsPlot.addSeries('rx data', {}, true)
                statsPlot.addSeries('tx data', {}, true)
            }
        }
    }


}

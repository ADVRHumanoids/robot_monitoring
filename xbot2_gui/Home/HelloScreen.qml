import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import Common
import LivePlot
import Main
import Font

MultiPaneResponsiveLayout {


    property ClientEndpoint client: undefined

    property int numEvents: 0

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


    ScrollView {

        property string iconText: 'Status'
        property string iconChar: MaterialSymbolNames.vitals

        id: scroll
        contentWidth: availableWidth

        Column {

            spacing: CommonProperties.geom.spacing

            SectionHeader {
                text: 'XBot2 GUI'
            }

            ServerStatusCard {
                id: srvStatus
                client: root.client
                width: scroll.availableWidth
            }

        }
    }

    ScrollView {

        property string iconText: 'Server log'
        property string iconChar: MaterialSymbolNames.log

        id: textScroll
        contentWidth: availableWidth
        contentHeight: consoleText.height

        Column {

            spacing: CommonProperties.geom.spacing

            SectionHeader {
                text: 'Server Output'
            }

            TextArea {

                id: consoleText

                width: textScroll.contentWidth
                color: "white"
                readOnly: true

                placeholderText: "Console output"
                wrapMode: TextEdit.Wrap

                textFormat: TextEdit.RichText

                font.pixelSize: 14
            }

        }



    }

    //    MaterialResponsiveGrid {

    //        id: mainGrid

    //        anchors.fill: parent

    //        SectionHeader {
    //            property int columnSpan: mainGrid.columns
    //            text: 'XBot2 GUI'
    //        }

    //        ServerStatusCard {
    //            id: srvStatus
    //            client: root.client
    //            onStatsUpdated: {
    //                let t = appData.getTimeNs()*1e-9 - statsPlot.t0
    //                statsPlot.addPoint(statsPlot.currSeries['rx data'],
    //                                   t, rxKbps)
    //                statsPlot.addPoint(statsPlot.currSeries['tx data'],
    //                                   t, txKbps)
    //                statsPlot.addPoint(statsPlot.currSeries['server rtt'],
    //                                   t, client.srvRtt)
    //            }
    //        }

    //        Card {

    //            name: 'Statistics'

    //            height: srvStatus.height

    //            property int columnSpan: 8

    //            toolButtons: [
    //                SmallToolButton {
    //                    text: '\uf021'
    //                    font.family: CommonProperties.fontAwesome.solid.family
    //                    onClicked: statsPlot.resetView()
    //                }
    //            ]

    //            frontItem: Plotter {
    //                id: statsPlot
    //                anchors.fill: parent
    //                property real t0: appData.getTimeNs()*1e-9
    //                axisLeftTitle: 'RTT [ms]'
    //                axisRightTitle: 'Data rate [kbps]'

    //            }

    //            Component.onCompleted: {
    //                statsPlot.addSeries('server rtt', {})
    //                statsPlot.addSeries('rx data', {}, true)
    //                statsPlot.addSeries('tx data', {}, true)
    //            }
    //        }
    //    }


}

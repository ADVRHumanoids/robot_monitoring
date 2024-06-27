import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Audio
import LivePlot

ScrollView {

    GridLayout {

        columns: 2
        // uniformCellHeights: true

        //
        Label {
            text: 'Audio Capture'
        }

        Switch {
            id: asrSwitch
            checked: false
            onClicked: AudioBroadcaster.active = checked

            Component.onCompleted: checked = AudioBroadcaster.active

            Connections {
                target: AudioBroadcaster
                function onActiveChanged() {
                    asrSwitch.checked = AudioBroadcaster.active
                }
                function onLevelChanged() {

                    // plot audio level

                    if(plot.levelTimeSeries === undefined) {
                        plot.levelTimeSeries = plot.addSeries('level', {}, false)
                        plot.levelTimeSeries.axisValue.min = 0
                        plot.levelTimeSeries.axisValue.max = 1.0
                        plot.initialTime = appData.getTimeNs() * 1e-9
                    }

                    plot.addPoint(plot.levelTimeSeries, appData.getTimeNs() * 1e-9 - plot.initialTime, AudioBroadcaster.level)
                }
            }
        }

        //
        Label {
            text: 'Audio Source'
        }

        ComboBox {
            model: AudioBroadcaster.devices
            onCurrentTextChanged: AudioBroadcaster.currentDevice = currentText
            Layout.fillWidth: true
        }

        //
        Label {
            text: `Audio Level (${AudioBroadcaster.level.toFixed(2)})`
        }

        Plotter {

            id: plot
            Layout.fillWidth: true

            plotterLegend: plotterLegend

            interactive: false

            chartView.title: 'Sound level'
            chartView.titleColor: palette.text
            chartView.margins {
                bottom: 6
                left: 6
                right: 6
                top: 6
            }

            property var levelTimeSeries: undefined

            property real initialTime: 0

            PlotterLegend {
                id: plotterLegend
                chart: plot.chartView
                visible:false
            }

        }
    }

}

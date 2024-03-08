import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import LivePlot
import "HorizonViz.js" as Logic

Control {

    function addSolutionTime(stamp, solTime) {

        if(plot.solutionTimeSeries === undefined) {
            plot.solutionTimeSeries = plot.addSeries('solution_time', {}, false)
            plot.solutionTimeSeries.axisValue.min = 0
            plot.solutionTimeSeries.axisValue.max = 0.1
        }

        if(plot.initialTime < 0) {
            plot.initialTime = stamp
        }

        plot.addPoint(plot.solutionTimeSeries, stamp - plot.initialTime, solTime)
    }

    function updateGaitPattern(timelines) {
        let i = 0
        for (const [name, tl] of Object.entries(timelines)) {

            let len = tl.dur.length

            for(let k = 0; k < len; k++) {

                if(k === 0 && len > 1) {
                    phaseViz.setPhaseData(i, k, tl.phases[k], tl.k0[k+1] - tl.dur[k], tl.dur[k])
                }
                else {
                    phaseViz.setPhaseData(i, k, tl.phases[k], tl.k0[k], tl.dur[k])
                }
            }

            phaseViz.setPhaseNumber(i, len)
            i++
        }
    }

    //
    id: root

    padding: 4

    contentItem: ColumnLayout {

        id: col

        anchors.fill: parent

        TabBar {

            Layout.fillWidth: true

            id: tabBar

            TabButton {
                text: 'Solution Time'
            }

            TabButton {
                text: 'Gait Pattern'
            }

        }

        StackLayout {

            Layout.fillHeight: true
            Layout.fillWidth: true

            currentIndex: tabBar.currentIndex



            Plotter {

                id: plot

                plotterLegend: plotterLegend

                interactive: false

                chartView.title: 'Solution Time [s]'
                chartView.titleColor: palette.text
                chartView.margins {
                    bottom: 6
                    left: 6
                    right: 6
                    top: 6
                }

                property var solutionTimeSeries: undefined

                property real initialTime: -1


            }

            PhaseViz {

                id: phaseViz
            }
        }

    }

    PlotterLegend {
        id: plotterLegend
        chart: plot.chartView
        visible:false
    }

}

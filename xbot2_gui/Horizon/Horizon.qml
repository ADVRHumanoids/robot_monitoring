import QtQuick

import Common
import Joy
import LivePlot
import Main

import "Horizon.js" as Logic

Item {

    property ClientEndpoint client

    property list<real> vref: [0, 0, 0, 0, 0, 0]

    // PhaseViz {
    //     anchors {
    //         top: parent.top
    //         margins: 20
    //         horizontalCenter: parent.horizontalCenter
    //     }
    //     width: 300
    //     height: 200
    // }

    Plotter {
        id: plot

        anchors {
            top: parent.top
            margins: 20
            horizontalCenter: parent.horizontalCenter
        }

        width: 400
        height: 250

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

    PlotterLegend {
        id: plotterLegend
        chart: plot.chartView
        anchors {
            top: plot.bottom
            right: plot.right
        }
        visible:false
    }

    HorizonControl {
        id: configPane
        anchors {
            top: parent.top
            right: parent.right
            margins: CommonProperties.geom.margins
        }
        onAlwaysWalkChanged: Logic.walkSwitch(alwaysWalk)
    }

    DualJoy {

        anchors {
            top: plot.bottom
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            margins: CommonProperties.geom.margins
        }

        leftPad.horizontalOnly: !configPane.joyXEnabled
        leftPad.verticalOnly: !configPane.joyYEnabled
        rightPad.horizontalOnly: true

        onLeftPadMoved: function(x, y ){
            vref[0] = y*configPane.maxSpeed
            vref[1] = -x*configPane.maxSpeed
            Logic.sendVref(vref)
        }

        onRightPadMoved: function(x, y ){
            vref[5] = -x*configPane.maxSpeed
            Logic.sendVref(vref)
        }

        onJoyPressedChanged: {

            if(configPane.alwaysWalk) {
                return
            }

            Logic.walkSwitch(joyPressed)
        }
    }

    Connections {

        target: client

        onObjectReceived: function(msg) {

            if(msg.type === 'horizon_status') {

                if(plot.solutionTimeSeries === undefined) {
                    plot.solutionTimeSeries = plot.addSeries('solution_time', {}, false)
                    plot.solutionTimeSeries.axisValue.min = 0
                    plot.solutionTimeSeries.axisValue.max = 0.1
                }

                if(plot.initialTime < 0) {
                    plot.initialTime = msg.stamp
                }

                console.log(plot.solutionTimeSeries.series)

                plot.addPoint(plot.solutionTimeSeries, msg.stamp - plot.initialTime, msg.solution_time)
            }

        }

    }

}

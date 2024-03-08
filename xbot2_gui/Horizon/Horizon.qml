import QtQuick

import Common
import Joy
import LivePlot
import Main

import "Horizon.js" as Logic

Item {

    LayoutClassHelper {
        id: layout
        targetWidth: parent.width
    }

    property ClientEndpoint client

    property list<real> vref: [0, 0, 0, 0, 0, 0]



    HorizonViz {

        id: viz

        anchors {
            top: parent.top
            margins: 20
            horizontalCenter: parent.horizontalCenter
        }

        width: parent.width - configPane.width - 300
        height: parent.height / 2
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

        compactLayout: layout.compact

        anchors {
            top: viz.bottom
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

                viz.addSolutionTime(msg.stamp, msg.solution_time)
                return
            }

            if(msg.type === 'horizon_timelines') {

                viz.updateGaitPattern(msg.timelines)
                return
            }
        }

    }

}

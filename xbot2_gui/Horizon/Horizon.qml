import QtQuick

import Common
import Joy

import "Horizon.js" as Logic

Item {

    property list<real> vref: [0, 0, 0, 0, 0, 0]

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
            top: configPane.bottom
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

}

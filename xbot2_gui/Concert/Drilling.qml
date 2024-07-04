import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import Main
import Video
import "../Video/VideoStream.js" as VideoStream
import Joy
import Common
import Font

import "Drilling.js" as Logic


Item {

    property ClientEndpoint client

    property string pageName

    signal pageSelected()

    id: root

    VideoStream {

        property string videoStreamName: '/drill_blob_detector/annotated_image_raw/theora'

        id: video

        z: -1

        anchors.fill: parent

        Connections {
            target: client
            function onTheoraPacketReceived(msg) {
                if(msg.stream_name === video.videoStreamName) {
                    video.setTheoraPacket(msg)
                }
            }
        }

        Component.onCompleted: {


        }

    }

    Pane {

        padding: 16

        id: progressPane

        anchors {
            top: parent.top
            horizontalCenter: parent.horizontalCenter
            margins: 16
        }

        background: Rectangle {
            property bool _completed: statusLabel.text === 'Completed'
            property bool _failed: statusLabel.text === 'Failed'
            color: _completed ? 'green' : (_failed ? 'red' : Qt.lighter(palette.window))
            opacity: 0.8
            radius: 4
        }

        contentItem: Row {
            // width: parent.width
            spacing: 16

            TextField {
                id: statusLabel
                placeholderText: ' Status'
                text: '--'
                font.pointSize: 16
                width: implicitWidth + 20
            }

            TextField {
                id: forceLabel
                placeholderText: 'Force (N)'
                text: '--'
                readOnly: true
                font.pointSize: 16
                width: implicitWidth + 20
            }

            TextField {
                id: errorLabel
                placeholderText: 'Error (mm)'
                text: '--'
                readOnly: true
                font.pointSize: 16
                width: implicitWidth + 20
            }

        }


    }

    Pane {

        padding: 4

        id: toolCol

        anchors {
            right: parent.right
            top: parent.top
            topMargin: CommonProperties.geom.spacing
        }

        background: Rectangle {
            color: Qt.lighter(palette.window)
            opacity: 0.8
            radius: 8
            clip: true
        }

        contentItem: DrillingControl {
            id: configPane

            onMotionModeChanged: {
                if(motionMode && motionTarget === 'Arm') {
                    console.log('enable arm control')
                    Logic.enableArmControl(true)
                }
                else {
                    console.log('disable arm control')
                    Logic.enableArmControl(false)
                }
            }

        }

    }


    DualJoy {

        property list<real> vref: [0, 0, 0, 0, 0, 0]
        property string baseTaskName: '[simple topic] /omnisteering/cmd_vel'
        property string armTaskName: configPane.armEE

        enabled: configPane.motionMode

        anchors {
            top: toolCol.bottom
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            margins: 16
        }

        leftPad.horizontalOnly: !configPane.joyXEnabled && !configPane.joyZEnabled
        leftPad.verticalOnly: !configPane.joyYEnabled
        rightPad.horizontalOnly: configPane.motionTarget === 'Base'
        rightPad.verticalOnly: configPane.motionTarget === 'Arm'


        onLeftPadMoved: function(x, y ){

            if(configPane.motionTarget === 'Base') {
                vref[0] = y*configPane.maxSpeed
                vref[1] = -x*configPane.maxSpeed
                Joy.sendVref(baseTaskName, vref)
            }

            if(configPane.motionTarget === 'Arm') {
                let verticalPadIdx = configPane.joyZEnabled ? 2 : 0
                let verticalPadSign = configPane.joyZEnabled ? -1 : 1
                vref[verticalPadIdx] = -y*configPane.maxSpeed*verticalPadSign
                vref[1] = -x*configPane.maxSpeed
                Joy.sendVref(armTaskName, vref)
            }

        }

        onRightPadMoved: function(x, y ){

            if(configPane.motionTarget === 'Base') {
                vref[5] = -x*configPane.maxSpeed
                Joy.sendVref(baseTaskName, vref)
            }

            if(configPane.motionTarget === 'Arm') {
                vref[4] = -y*configPane.maxSpeed
                Joy.sendVref(armTaskName, vref)
            }

        }

    }

    Connections {

        target: client

        onObjectReceived: function(msg) {

            if(msg.type === 'concert_drill_progress') {

                statusLabel.text = msg.status

                forceLabel.text = msg.force
                errorLabel.text = (1000*Math.sqrt(msg.error_x*msg.error_x + msg.error_y*msg.error_y)).toFixed(1)
            }

            if(msg.type === 'concert_blob_array') {

                let blob_list = []

                for(let b of msg.blobs) {
                    blob_list.push(b.id)
                }

                configPane.blobIds = blob_list
            }

        }

    }

    onPageSelected: {
        VideoStream.setStream(video.videoStreamName, video)
        Logic.updateTaskNames()
    }

}

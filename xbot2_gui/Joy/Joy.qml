import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Main
import Video
import "../Video/VideoStream.js" as VideoStream
import "Joy.js" as Joy
import Common

Item {

    property ClientEndpoint client

    JoyCartesianCard {

        id: setupCard

        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
            margins: CommonProperties.geom.spacing
        }

        onVideoStreamChanged: {
            VideoStream.setStream(videoStream, video)
        }

        backgroundColor: Qt.rgba(0, 0, 0, 0.5)
    }

    VideoStream {

        id: video

        z: -1

        anchors.fill: parent

        Connections {
            target: client
            function onTheoraPacketReceived(msg) {
                if(msg.stream_name === setupCard.videoStream) {
                    video.setTheoraPacket(msg)
                }
            }
        }

    }

    property var vref: [0, 0, 0, 0, 0, 0]
    property alias maxLinearV: setupCard.maxSpeed
    property alias maxAngularV: setupCard.maxSpeed
    property alias currentTask: setupCard.currentTask

    Pad  {
        anchors {
            left: parent.left
            bottom: parent.bottom
            margins: 48
        }

        verticalOnly: setupCard.linXOnly

        side: Math.min(200, parent.width/2 - 64)

        onJoystickMoved: function (x, y) {
            console.log(`${x} ${y}`)
            vref[0] = y*maxLinearV
            vref[1] = -x*maxLinearV
            Joy.sendVref(currentTask, vref)
        }

    }

    Pad  {

        anchors {
            right: parent.right
            bottom: parent.bottom
            margins: 48
        }

        horizontalOnly: true

        side: Math.min(200, parent.width/2 - 64)

        onJoystickMoved: function(x, y) {
            console.log(`${x} ${y}`)
            vref[5] = -x*maxAngularV
            Joy.sendVref(currentTask, vref)
        }
    }
}


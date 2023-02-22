import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import ".."
import "../Video"
import "../Video/VideoStream.js" as VideoStream
import "Joy.js" as Joy
import xbot2_gui.common

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
    }


    GridLayout {

        anchors{
            left: parent.left
            right: parent.right
            top: setupCard.bottom
            bottom: parent.bottom
            margins: CommonProperties.geom.spacing
        }
        columns: width > height ? 2 : 1
        columnSpacing: 16
        rowSpacing: 16

        Repeater {

            model: 2

            VideoStreamCard {

                id: video

                Layout.fillHeight: !hidden
                Layout.fillWidth: true
                Layout.preferredWidth: 1

                Component.onCompleted: VideoStream.refreshNames(video)

                onUpdateAvailableStreamIds: VideoStream.refreshNames(video)

                onStreamIdChanged: {
                    VideoStream.setStream(streamId, video)
                    streamName = streamId
                }

                property bool _constructed: false

                onAvailableStreamIdsChanged: {
                    if(!_constructed) {
                        console.log(`setting idx = ${settings.streamIndex}`)
                        currentIndex = settings.streamIndex
                        streamName = settings.streamName
                        _constructed = true
                    }
                }

                Connections {
                    target: client
                    function onTheoraPacketReceived(msg) {
                        if(msg.stream_name === streamId) {
                            video.setTheoraPacket(msg)
                        }
                    }
                }

                Settings {
                    id: settings
                    category: 'videoStreamCard' + index
                    property int streamIndex: 0
                    property string streamName: 'Stream Name'
                }

                Component.onDestruction: {
                    console.log(`saving settings ` + index)
                    settings.streamIndex = video.currentIndex
                    settings.streamName = video.streamName
                    settings.sync()
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

        side: Math.min(200, parent.width/2 - 64)

        opacity: 0.7

        onJoystickMoved: {
            vref[0] = joyY*maxLinearV
            vref[1] = -joyX*maxLinearV
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

        opacity: 0.7

        onJoystickMoved: {
            vref[5] = -joyX*maxAngularV
            Joy.sendVref(currentTask, vref)
        }
    }
}


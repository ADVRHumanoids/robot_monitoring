import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import xbot2_gui.Common
import "../Common"
import "../Joy"
import "../Video"
import "../Video/VideoStream.js" as VideoStream
import ".."



Item {

    property ClientEndpoint client

    id: root

    RowLayout {

        id: row

        anchors.fill: parent
        anchors.margins: 20
        spacing: 30


        VideoStreamCard {

            id: video

            Layout.fillHeight: true
            Layout.fillWidth: true

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

        Pad {

            property var vref: [0, 0, 0, 0, 0, 0]

            Layout.fillHeight: true
            Layout.preferredWidth: root.width/3

            onJoystickMoved: function(x, y)
            {
                vref[0] = x
                vref[1] = y

                var msg = {
                    'type': 'concert_drill_vref',
                    'vref': vref
                }

                client.sendTextMessage(JSON.stringify(msg))

                vref = [0, 0, 0, 0, 0, 0]
            }
        }

    }

}

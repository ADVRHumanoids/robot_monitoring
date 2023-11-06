import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import Common
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
//                    console.log(`setting idx = ${settings.streamIndex}`)
//                    currentIndex = settings.streamIndex
//                    streamName = settings.streamName
                    _constructed = true
                }
            }

            Connections {
                target: root.client
                function onTheoraPacketReceived(msg) {
                    if(msg.stream_name === video.streamId) {
                        video.setTheoraPacket(msg)
                    }
                }
            }

//            Settings {
//                id: settings
//                category: 'videoStreamCard' + index
//                property int streamIndex: 0
//                property string streamName: 'Stream Name'
//            }

            Component.onDestruction: {
//                console.log(`saving settings ` + index)
//                settings.streamIndex = video.currentIndex
//                settings.streamName = video.streamName
//                settings.sync()
            }
        }

        ColumnLayout {

            Layout.fillHeight: true
            Layout.preferredWidth: root.width/3
            spacing: 50

            RowLayout {

                Layout.fillWidth: true
                Layout.margins: 20

                DelayButton {
                    text: ' Drill '
                    font.pointSize: 20
                    delay: 1000

                    onActivated: {
                       client.doRequest('POST', '/concert/do_drill', '')
                    }
                }

                Item {
                    Layout.fillWidth: true
                }

                DelayButton {
                    text: ' Align '
                    font.pointSize: 20
                    delay: 1000

                    onActivated: {
                       client.doRequest('POST', '/concert/do_align', '')
                    }
                }

                Item {
                    Layout.fillWidth: true
                }

                Switch {
                    id: controlSwitch
                    text: 'Control'
                    font.pointSize: 20

                    onCheckedChanged: {
                        if(checked) {
                            client.doRequest('POST', '/concert/enable_control', '')
                        }
                        else {
                            client.doRequest('POST', '/concert/disable_control', '')
                        }
                    }
                }

            }


            Pad {

                enabled: controlSwitch.checked
                opacity: enabled ? 1 : 0.3
                xLabel: 'Y'
                yLabel: 'Z'

                property var vref: [0, 0, 0, 0, 0, 0]

                Layout.fillHeight: true
                Layout.fillWidth: true

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

            TextArea {
                Layout.alignment: Qt.AlignHCenter
                text: 'Press ALIGN for one second to move the robot in the pre-drilling position. \nToggle Control to enable EE drill position fine tuning. \nPress DRILL for one second to make a hole.'
                wrapMode: TextArea.WordWrap
            }

        }

    }

}

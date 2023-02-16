import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import ".."
import "VideoStream.js" as Logic

Item {

    property ClientEndpoint client

    GridLayout {
        anchors.fill: parent
        columns: 2
        columnSpacing: 16
        rowSpacing: 16

        Repeater {
            model: 2
            VideoStreamCard {
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.preferredWidth: 1
                id: video
                Component.onCompleted: {
                    Logic.refreshNames(video)
                }
                onStreamIdChanged: {
                    Logic.setStream(streamId, video)
                }
                Connections {
                    target: client
                    onTheoraPacketReceived: (msg) => {
                        if(msg.stream_name === streamId) {
                            video.setTheoraPacket(msg)
                        }
                    }
                }
            }
        }

    }
}


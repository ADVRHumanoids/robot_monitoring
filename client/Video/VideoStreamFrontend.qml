import QtQuick 2.0
import QtQuick.Layouts
import QtQuick.Controls
import NextUiModules

import ".."
import "video.js" as Logic

Item {

    property ClientEndpoint client: undefined

    property alias video: video

    VideoStream {

        id: video

        anchors.fill: parent

        onVideoStreamChanged: (name) => Logic.setStream(name)

        onRefreshNamesRequested: Logic.refreshNames()

    }

    Component.onCompleted: {

        client.theoraPacketReceived.connect(function(msg) {
            if(msg.stream_name === video.streamName) {
                video.setTheoraPacket(msg)
            }
        })

    }


}

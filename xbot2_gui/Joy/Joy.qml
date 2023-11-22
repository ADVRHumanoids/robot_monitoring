import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Main
import Video
import "../Video/VideoStream.js" as VideoStream
import "Joy.js" as Joy
import Common
import Font

Item {

    LayoutClassHelper {
        id: layout
        targetWidth: parent.width
    }
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

    Column {

        id: toolCol

        spacing: 3

        anchors {
            right: setupCard.right
            top: setupCard.bottom
            topMargin: CommonProperties.geom.spacing
        }

        Label {
            text: ' max speed'
            font.pointSize: 10
        }

        DoubleSpinBox {
            id: maxSpeedLinearSpinBox
            from: 0.0
            to: 2.0
            value: 0.2
        }

        Item {
            width: parent.width
            height: 6
        }

        Label {
            text: ' enabled directions'
            font.pointSize: 10
        }

        Row {
            CheckBox {
                id: chkX
                text: 'X'
                checked: true
            }
            CheckBox {
                id: chkY
                text: 'Y'
                checked: true
            }
        }

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
    property alias maxLinearV: maxSpeedLinearSpinBox.value
    property alias maxAngularV: maxSpeedLinearSpinBox.value
    property alias currentTask: setupCard.currentTask

    RowLayout {

        visible: !layout.compact

        anchors {
            left: parent.left
            bottom: parent.bottom
            right: parent.right
            top: toolCol.bottom
            margins: 16
        }

        LayoutItemProxy {
            target: leftPad
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: parent.width / 2
        }

        Item {
            Layout.fillWidth: true
            Layout.minimumWidth: 50
        }

        LayoutItemProxy {
            target: rightPad
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: parent.width / 2
        }

    }

    RowLayout {

        visible: layout.compact

        anchors {
            left: parent.left
            bottom: parent.bottom
            right: parent.right
            top: toolCol.bottom
            margins: 16
        }

        TabButton {
            text: MaterialSymbolNames.arrowBack
            font.family: MaterialSymbolNames.filledFont.font.family
            onClicked: padSwipe.decrementCurrentIndex()
            font.pointSize: 20
            z: -1
            enabled: padSwipe.currentIndex > 0
        }

        SwipeView {

            id: padSwipe

            interactive: false

            Layout.fillWidth: true
            Layout.fillHeight: true

            LayoutItemProxy {
                target: leftPad
                visible: padSwipe.currentIndex === 0
            }

            LayoutItemProxy {
                target: rightPad
                visible: padSwipe.currentIndex === 1
            }

        }

        TabButton {
            text: MaterialSymbolNames.arrowForward
            font.family: MaterialSymbolNames.filledFont.font.family
            font.pointSize: 20
            onClicked: padSwipe.incrementCurrentIndex()
            z: -1
            enabled: padSwipe.currentIndex + 1 < padSwipe.count
        }

    }

    Pad  {

        id: leftPad

        verticalOnly: !chkY.checked
        horizontalOnly: !chkX.checked

        side: 200

        backgroundColor: 'transparent'

        onJoystickMoved: function (x, y) {
            console.log(`${x} ${y}`)
            vref[0] = y*maxLinearV
            vref[1] = -x*maxLinearV
            Joy.sendVref(currentTask, vref)
        }

    }

    Pad  {

        id: rightPad

        horizontalOnly: true

        side: 200

        backgroundColor: 'transparent'

        onJoystickMoved: function(x, y) {
            console.log(`${x} ${y}`)
            vref[5] = -x*maxAngularV
            Joy.sendVref(currentTask, vref)
        }
    }
}


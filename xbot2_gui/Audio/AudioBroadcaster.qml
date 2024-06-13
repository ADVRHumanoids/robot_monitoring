pragma Singleton

import QtQuick
import QtMultimedia
import QtQuick.Controls
import QtTextToSpeech

import LivePlot
import Main

Item {

    property alias currentDevice: audioSrc.currentDevice

    property alias devices: audioSrc.devices

    property alias level: audioSrc.level

    property alias bytesAvailable: audioSrc.bytesAvailable

    property alias enableSend: audioSrc.enableSend

    signal readyRead()

    property alias active: audioSrc.active

    function start() {
        audioSrc.start()
    }

    function stop() {
        audioSrc.stop()
    }

    function readBase64(size) {
        return audioSrc.readBase64(size)
    }

    //
    id: root

    AudioSource {

        id: audioSrc

        property bool enableSend: true

        onReadyRead: {

            root.readyRead()
        }

    }

}

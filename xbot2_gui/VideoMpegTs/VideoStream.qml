import QtQuick
import QtMultimedia
import Network

Item {

    property bool autoPlay: true

    function setMpegTsDatagram(msg) {
        const skipped = msg.seq - _lastSeq - 1
        if(_lastSeq !== -1 && skipped !== 0) {
            console.log(`WARN: skipped ${skipped} MPEG-TS datagrams`)
        }
        _lastSeq = msg.seq
        sock.sendBinaryMessage(msg.data)
    }

    readonly property string errorString: player.errorString

    signal streamReady()

    signal streamError(string message)

    //
    id: root
    implicitHeight: videoOutput.implicitHeight
    implicitWidth: videoOutput.implicitWidth

    property string _mediaPlayerAddress: "127.0.0.1"
    property int _mediaPlayerPort: 12345
    property int _lastSeq: -1

    function _restart() {
        retryTimer.stop()
        player.stop()

        // Reassigning the source makes QMediaPlayer reopen FFmpeg's UDP input.
        // Merely calling play() after a network timeout can leave the old
        // demuxer in its terminal error state.
        player.source = ""
        player.source = `udp://${_mediaPlayerAddress}:${_mediaPlayerPort}?reuse=1&fifo_size=512&buffer_size=65536&timeout=3000000`
        if (root.autoPlay)
            player.play()
    }

    //
    UdpSocket {
        id: sock
        hostname: _mediaPlayerAddress
        port: _mediaPlayerPort
    }

    //
    MediaPlayer {
        id: player
        videoOutput: videoOutput

        // Qt 6.10's low-latency intent tells the FFmpeg backend that keeping a
        // large, smooth playback reservoir is less important than showing the
        // newest live frame. A small probe still contains several MPEG-TS PAT,
        // PMT, SPS and PPS repetitions from the sender.
        playbackOptions.playbackIntent:
            PlaybackOptions.PlaybackIntent.LowLatencyStreaming
        playbackOptions.probeSize: 2048

        onMediaStatusChanged: {
            if (mediaStatus === MediaPlayer.LoadedMedia
                    || mediaStatus === MediaPlayer.BufferedMedia) {
                root.streamReady()
                if (root.autoPlay && playbackState !== MediaPlayer.PlayingState)
                    play()
            } else if (mediaStatus === MediaPlayer.EndOfMedia
                       || mediaStatus === MediaPlayer.InvalidMedia) {
                retryTimer.restart()
            }
        }

        onErrorOccurred: function(error, description) {
            root.streamError(description)
            retryTimer.restart()
        }
    }

    //
    VideoOutput {
        id: videoOutput
        anchors.fill: parent
        fillMode: VideoOutput.PreserveAspectFit
        endOfStreamPolicy: VideoOutput.KeepLastFrame
    }

    //
    Timer {
        id: retryTimer
        interval: 1000
        repeat: false
        onTriggered: root._restart()
    }

    //
    Component.onCompleted: {
        if (autoPlay)
            _restart()
    }

}

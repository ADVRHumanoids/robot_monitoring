import QtQuick
import QtQuick.Controls
import QtMultimedia
import Network

Item {

    property alias source: player.source

    property bool autoPlay: true

    readonly property string errorString: player.errorString

    signal streamReady()

    signal streamError(string message)

    function restart() {
        retryTimer.stop()
        player.stop()

        // Reassigning the source makes QMediaPlayer reopen FFmpeg's UDP input.
        // Merely calling play() after a network timeout can leave the old
        // demuxer in its terminal error state.
        const _source = player.source
        player.source = ""
        player.source = _source
    }

    onSourceChanged: {
        if(source === '') player.stop()
        if(autoPlay) player.play()
    }

    //
    id: root
    implicitHeight: videoOutput.implicitHeight
    implicitWidth: videoOutput.implicitWidth

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
    Column {
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        Label {
            text: `Source: ${player.source}`
        }
        Label {
            id: statusLabel
        }
    }

    //
    Timer {
        id: retryTimer
        interval: 1000
        repeat: false
        onTriggered: root.restart()
    }

    //
    Component.onCompleted: {
        if (autoPlay)
            restart()
    }

    onStreamReady: statusLabel.text = 'Stream ready'
    onStreamError: statusLabel.text = 'Stream error: ' + message

}

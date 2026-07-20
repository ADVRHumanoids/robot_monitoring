import QtQuick
import QtMultimedia

Item {
    id: root

    /*! Local interface used by FFmpeg's UDP receiver. */
    property string bindAddress: "0.0.0.0"

    /*! UDP port shared with qt_h264_udp_sender. */
    property int port: 5000

    /*! Start playback when the component becomes ready. */
    property bool autoPlay: true

    /*! Human-readable state suitable for a GUI status label. */
    readonly property string statusText: {
        if (player.error !== MediaPlayer.NoError)
            return player.errorString
        switch (player.mediaStatus) {
        case MediaPlayer.NoMedia: return "Waiting to start"
        case MediaPlayer.LoadingMedia: return "Waiting for UDP stream"
        case MediaPlayer.LoadedMedia: return "Stream found"
        case MediaPlayer.BufferingMedia: return "Buffering"
        case MediaPlayer.BufferedMedia: return "Playing"
        case MediaPlayer.StalledMedia: return "Stream stalled"
        case MediaPlayer.EndOfMedia: return "Stream ended"
        case MediaPlayer.InvalidMedia: return "Invalid stream"
        default: return "Connecting"
        }
    }

    readonly property string errorString: player.errorString
    readonly property url streamUrl: "udp://" + bindAddress + ":" + port
        + "?reuse=1&fifo_size=512&buffer_size=65536&timeout=3000000"

    signal streamReady()
    signal streamError(string message)

    function restart() {
        retryTimer.stop()
        player.stop()

        // Reassigning the source makes QMediaPlayer reopen FFmpeg's UDP input.
        // Merely calling play() after a network timeout can leave the old
        // demuxer in its terminal error state.
        player.source = ""
        player.source = root.streamUrl
        if (root.autoPlay)
            player.play()
    }

    onStreamUrlChanged: restart()

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

    VideoOutput {
        id: videoOutput
        anchors.fill: parent
        fillMode: VideoOutput.PreserveAspectFit
        endOfStreamPolicy: VideoOutput.KeepLastFrame
    }

    Timer {
        id: retryTimer
        interval: 1000
        repeat: false
        onTriggered: root.restart()
    }

    Component.onCompleted: {
        if (autoPlay)
            restart()
    }
}

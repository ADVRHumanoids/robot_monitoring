import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Video
import Common

Item {

    // public
    function setTheoraHeader(hdr) {
        for(let i = 0; i < 3; i++) {
            console.log(`got bos=${hdr[i].bOS} eos=${hdr[i].eOS} pktno=${hdr[i].packetno}`)
            video.setTheoraPacket(hdr[i].data,
                                  hdr[i].bOS,
                                  hdr[i].eOS,
                                  hdr[i].granulepos,
                                  hdr[i].packetno)
        }
        _hdr_recv = true
    }

    function setTheoraPacket(msg) {

        if(!_hdr_recv) {
            return
        }

        video.setTheoraPacket(msg.data,
                              msg.bOS,
                              msg.eOS,
                              msg.granulepos,
                              msg.packetno)
    }

    // private
    id: root

    property bool _hdr_recv: false
    property string streamName

    // force video painter to respect the source aspect ratio
    AspectRatio {

        anchors.fill: parent
        aspectRatio: video.implicitWidth/video.implicitHeight

        VideoStreamPainter {
            id: video
            anchors.fill: parent
        }
    }
}


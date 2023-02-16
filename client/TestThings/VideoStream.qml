import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import NextUiModules


Item {

    // public
    function setTheoraHeader(hdr) {
        for(let i = 0; i < 3; i++) {
            video.setTheoraPacket(hdr[i].data,
                                  hdr[i].b_o_s,
                                  hdr[i].e_o_s,
                                  hdr[i].granulepos,
                                  hdr[i].packetno)
        }
        _hdr_recv = true
        console.log(streamName + ': set headers done')
    }

    function setTheoraPacket(msg) {

        if(!_hdr_recv) {
            return
        }

        video.setTheoraPacket(msg.data,
                              msg.b_o_s,
                              msg.e_o_s,
                              msg.granulepos,
                              msg.packetno)
    }

    // private
    id: root

    property bool _hdr_recv: false

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


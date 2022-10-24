import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material
import NextUiModules

Page {

    property var client: undefined

    Row {

        anchors.centerIn: parent
        spacing: 50

        Pad {

            onJoystickMoved: function (x, y) {
                console.log(x + ', ' + y)
            }
        }

        Rectangle {
            width: 640
            height: 480
            color: "transparent"
            VideoStreamPainter {
                id: video
                anchors.fill: parent
                anchors.margins: 5
            }
        }


        Pad {

            xLabel: 'YAW'

            horizontalOnly: true

            onJoystickMoved: function (x, y) {
                console.log(x + ', ' + y)
            }
        }

        Pad {

            yLabel: 'Z'

            verticalOnly: true

            onJoystickMoved: function (x, y) {
                console.log(x + ', ' + y)
            }
        }

    }

    Component.onCompleted: {

//        client.jpegReceived.connect(function(msg) {
//            video.setImage(msg.data)
//            video.update()
//        })

        client.theoraPacketReceived.connect(function(msg) {
            video.setTheoraPacket(msg.data,
                                  msg.b_o_s,
                                  msg.e_o_s,
                                  msg.granulepos,
                                  msg.packetno)
        })

        client.doRequest('GET', '/theora_header', {},
                         function(msg) {

                             if(!msg.success) {
                                 console.log('could not fetch headers')
                             }

                             for(let i = 0; i < 3; i++) {
                                 let hdr = msg.hdr[i]
                                 video.setTheoraPacket(hdr.data,
                                                       hdr.b_o_s,
                                                       hdr.e_o_s,
                                                       hdr.granulepos,
                                                       hdr.packetno)
                             }

                             console.log('set headers!!!')

                         }
                         )
    }

}

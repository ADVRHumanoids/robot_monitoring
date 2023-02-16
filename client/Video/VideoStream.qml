import QtQuick 2.0
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Controls.Material

import NextUiModules
import "../TestThings"


Rectangle {

    id: root
    color: Qt.lighter(Material.backgroundColor)
    radius: 4

    property alias names: combo.model
    property alias streamName: combo.currentText
    property bool _hdr_recv: false

    signal refreshNamesRequested

    signal videoStreamChanged(string name)

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

        if(!_hdr_recv ||! activeCheck.checked) {
            return
        }


        video.setTheoraPacket(msg.data,
                              msg.b_o_s,
                              msg.e_o_s,
                              msg.granulepos,
                              msg.packetno)
    }

    ColumnLayout {

        id: col
        anchors.fill: parent
        anchors.margins: 16

        Label {
            text: "Video Stream"
            bottomPadding: 16
            font.pixelSize: 28
        }

        RowLayout {

            Layout.fillWidth: true
            spacing: 16

            Label {
                text: 'Camera'
            }

            ComboBox {
                id: combo
                enabled: count > 0
                Layout.fillWidth: true

                onCurrentTextChanged: {
                    if(activeCheck.checked) {
                        _hdr_recv = false
                        videoStreamChanged(currentText)
                    }
                }

            }

            Button {
                text: "Refresh"
                onReleased: {
                    refreshNamesRequested()
                }
            }

            CheckBox {
                id: activeCheck
                text: "Active"
                checked: true

                onCheckedChanged: {
                    if(checked && combo.enabled) {
                        _hdr_recv = false
                        videoStreamChanged(combo.currentText)
                    }
                    if(!checked) {
                        _hdr_recv = false
                        videoStreamChanged("")
                    }
                }
            }
        }

        AspectRatio {

            Layout.fillWidth: true
            Layout.fillHeight: true
            aspectRatio: video.implicitWidth/video.implicitHeight

            VideoStreamPainter {
                id: video
                anchors.fill: parent
            }
        }
    }

    Component.onCompleted: {
        refreshNamesRequested()
    }
}

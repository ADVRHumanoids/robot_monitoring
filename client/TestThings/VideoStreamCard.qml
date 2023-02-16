import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Card {

    property alias streamName: nameText.text
    property alias streamId: idCombo.currentText
    property alias availableStreamIds: idCombo.model

    function setTheoraHeader(hdr) {
        video.setTheoraHeader(hdr)
    }

    function setTheoraPacket(pkt) {
        video.setTheoraPacket(pkt)
    }

    // private
    id: root
    name: streamName

    frontItem: VideoStream {
        id: video
        anchors.fill: parent
    }

    backItem: GridLayout {
        anchors.fill: parent
        columns: 2
        columnSpacing: 16

        Label { text: 'ID' }
        ComboBox {
            Layout.fillWidth: true
            id: idCombo
        }

        Label { text: 'Name' }
        TextField {
            id: nameText
        }

        Item {
            Layout.fillHeight: true
        }
    }
}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import ".."

Card {

    property alias streamName: nameText.text
    property alias streamId: idCombo.currentText
    property alias availableStreamIds: idCombo.model
    property alias currentIndex: idCombo.currentIndex

    function setTheoraHeader(hdr) {
        video.setTheoraHeader(hdr)
    }

    function setTheoraPacket(pkt) {
        video.setTheoraPacket(pkt)
    }

    signal updateAvailableStreamIds()

    // private
    id: root
    name: streamName

    frontItem: VideoStream {
        id: video
        anchors.fill: parent
    }

    backItem: GridLayout {
        anchors.fill: parent
        columns: 3
        columnSpacing: 16

        Label { text: 'ID' }
        ComboBox {
            Layout.fillWidth: true
            id: idCombo
        }
        SmallToolButton {
            text: '\uf021'
            font.family: CommonProperties.fontAwesome.solid.family
            onClicked: root.updateAvailableStreamIds()
        }

        Label { text: 'Name' }
        TextField {
            Layout.fillWidth: true
            Layout.columnSpan: 2
            id: nameText
            placeholderText: 'Insert human-readable name'
        }

        Item {
            Layout.columnSpan: 3
            Layout.fillHeight: true
        }
    }
}

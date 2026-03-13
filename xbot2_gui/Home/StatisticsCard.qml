import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main

Card1 {

    property ClientEndpoint client

    //
    id: root

    property real bytesAll: 0
    property real bytesDelta: 0

    configurable: false
    collapsable: true
    collapsed: true

    name: 'Network Statistics'

    frontItem: Control {
        topPadding: 6
        anchors.fill: parent
        contentItem: GridLayout {

            id: grid

            columns: Math.ceil(width / 300)

            columnSpacing: CommonProperties.geom.spacing * 1.33
            rowSpacing: CommonProperties.geom.spacing * 1.33

        }
    }

    toolButtons: [
        Switch {
            id: showTotalSwitch
            text: 'Show total'
            checked: false
        }

    ]

    Timer {
        id: timer
        running: true
        repeat: true
        onTriggered: {

            let brc = client.bytesRecvCounters
            let numKeys = Object.keys(brc).length

            root.bytesDelta = brc.all/1024. - bytesAll
            root.bytesAll = brc.all/1024.

            if(grid.children.length !== numKeys)
            {
                grid.children = []

                for(const key of Object.keys(brc)) {
                    framedValue.createObject(grid, {'title': key})
                }
            }
        }
    }

    property Component rowHdr: Label {

    }

    property Component framedValue: FramedValue {
        id: comp
        property int oldMsgs: 0
        property real oldKbytes: 0
        Layout.fillWidth: true
        Layout.fillHeight: true
        Connections {
            target: timer
            function onTriggered() {

                // msgs
                let msgs = client.numMsgCounters[title]
                if(showTotalSwitch.checked) {
                    comp.value1 = `${msgs} msgs`
                }
                else {
                    comp.value1 = `${msgs - comp.oldMsgs} msgs`
                }
                comp.oldMsgs = msgs

                // bandwidth
                let kbytes = client.bytesRecvCounters[title] / 1024.
                if(showTotalSwitch.checked) {
                    comp.value2 = `${(kbytes*8).toFixed(1)} kb  (${(kbytes/root.bytesAll*100).toFixed(0)} %)`
                }
                else {
                    let delta = kbytes - comp.oldKbytes
                    comp.value2 = `${(delta*8).toFixed(1)} kbps  (${(delta/root.bytesDelta*100).toFixed(0)} %)`
                }
                comp.oldKbytes = kbytes
            }
        }
    }

}

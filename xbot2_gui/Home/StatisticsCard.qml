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

    frontItem: GridLayout {

        id: grid

        columns: 3

        columnSpacing: 8
        rowSpacing: 8

        anchors.fill: parent

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

            if(grid.children.length !== numKeys * 3)
            {
                grid.children = []

                for(const key of Object.keys(brc)) {
                    rowHdr.createObject(grid, {'text': key})
                    numField.createObject(grid, {'text': '--', 'fieldName': key})
                    kBField.createObject(grid, {'text': '--', 'fieldName': key})
                }
            }
        }
    }

    property Component rowHdr: Label {

    }

    property Component numField: TextField {
        id: numField
        property string fieldName
        property int oldValue: 0
        readOnly: true
        placeholderText: 'num msg'
        Layout.preferredHeight: 40
        Layout.minimumWidth: 100
        Layout.fillWidth: true
        Connections {
            target: timer
            function onTriggered() {
                let value = client.numMsgCounters[fieldName]
                if(showTotalSwitch.checked) {
                    numField.text = value
                }
                else {
                    numField.text = value - numField.oldValue
                }
                numField.oldValue = value
            }
        }
    }

    property Component kBField: TextField {
        id: kBField
        property string fieldName
        property real oldValue: 0
        readOnly: true
        placeholderText: 'bandwidth'
        Layout.preferredHeight: 40
        Layout.minimumWidth: 100
        Layout.fillWidth: true
        Connections {
            target: timer
            function onTriggered() {
                let value = client.bytesRecvCounters[fieldName] / 1024.
                if(showTotalSwitch.checked) {
                    kBField.text = `${(value*8).toFixed(1)} kb  (${(value/root.bytesAll*100).toFixed(0)} %)`
                }
                else {
                    let delta = value - kBField.oldValue
                    kBField.text = `${(delta*8).toFixed(1)} kbps  (${(delta/root.bytesDelta*100).toFixed(0)} %)`
                }
                kBField.oldValue = value
            }
        }
    }

}

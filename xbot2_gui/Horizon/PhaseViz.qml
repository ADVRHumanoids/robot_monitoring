import QtQuick
import QtQuick.Layouts
import Common

Item {

    property int horizonLength: 40
    property int numContacts: 4
    property var colorMap: {
        'stance': 'crimson',
        'flight': 'cadetblue',
        '--': 'grey'
    }

    function setPhaseData(timeline, idx, name, k0, duration) {
        // console.log(`adding phase ${timeline}[${idx}] ${name} ${k0} ${duration}`)
        rowRepeater.itemAt(timeline).setPhaseData(idx, name.substring(0, 6), k0, duration)
    }

    function setPhaseNumber(timeline, num) {
        rowRepeater.itemAt(timeline).setPhaseNumber(num)
    }

    //
    id: root
    implicitHeight: col.implicitHeight
    implicitWidth: col.implicitWidth
    clip: true


    property Component phaseComponent: Rectangle {

        property string name: '--'
        property int duration: 1
        property int last_x: 0

        height: parent !== null ? parent.height : 0
        width: root.width / root.horizonLength * duration
        color: Qt.alpha(root.colorMap[name], 0.2)
        radius: 4
        border.color: root.colorMap[name]
        visible: false

    }

    property Component timelineComponent: Item {

        // implicitHeight: row.implicitWidth
        // implicitWidth: row.implicitWidth
        required property int index
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.preferredHeight: 1

        function setPhaseData(idx, name, k0, duration) {

            if(idx >= phaseRepeater.count) {
                phaseRepeater.model = idx + 1
            }

            let phase = phaseRepeater.itemAt(idx)

            if(!phase.visible) {
                phase.x = k0 / root.horizonLength * width
            }
            else {
                let alpha = 1
                phase.x = phase.last_x + alpha*(k0 / root.horizonLength * width - phase.last_x)
            }

            phase.last_x = phase.x
            phase.name = name
            phase.duration = duration
            phase.visible = true
        }

        function setPhaseNumber(num) {
            for(let i = num; i < phaseRepeater.count; i++) {
                phaseRepeater.itemAt(i).visible = false
            }
        }


        Repeater {
            id: phaseRepeater
            delegate: root.phaseComponent
            model: 10
        }

    }

    ColumnLayout {

        id: col

        anchors.fill: parent

        Repeater {
            id: rowRepeater
            model: root.numContacts
            delegate: root.timelineComponent
        }

    }



}

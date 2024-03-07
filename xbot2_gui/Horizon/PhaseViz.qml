import QtQuick
import QtQuick.Layouts
import Common

Item {

    property real horizonLength: 1.0
    property int numContacts: 4

    function addPhase(i, name, dur) {
        console.log(i, name, dur)
        let obj = phaseComponent.createObject(null, {'name': name, 'duration': dur})
        rowRepeater.itemAt(i).add(obj, _time + horizonLength)
        _insert_time += dur
    }

    Timer {
        interval: 16
        repeat: true
        running: true
        onTriggered: {
            root._time += interval * 0.001
        }
    }

    //
    id: root
    implicitHeight: col.implicitHeight
    implicitWidth: col.implicitWidth
    clip: true
    property real _time: 0
    property real _insert_time: 0

    DebugRectangle {
        target: parent
    }

    property Component phaseComponent: Rectangle {

        required property string name
        required property real duration

        height: parent !== null ? parent.height : 0
        width: root.width / root.horizonLength * duration
        color: 'lightgreen'
        radius: 4

        Text {
            anchors.centerIn: parent
            text: name
        }

    }

    property Component timelineComponent: Item {

        // implicitHeight: row.implicitWidth
        // implicitWidth: row.implicitWidth
        required property int index
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.preferredHeight: 1

        function add(object, t0) {
            inner.children.push(object)
            object.x = t0 / root.horizonLength * root.width
        }

        Item {
            id: inner
            x: -root._time / root.horizonLength * root.width
            height: parent.height
        }

        // Row {
        //     id: row
        //     height: parent.height
        //     x: root._offset
        //     spacing: 1
        //     add: Transition {
        //         NumberAnimation {
        //             from: width + 100
        //             properties: "x"
        //             easing.type: Easing.OutQuad
        //         }
        //     }

        // }

        MouseArea {
            anchors.fill: parent
            onClicked: addPhase(index, 'Cose', 0.2)
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

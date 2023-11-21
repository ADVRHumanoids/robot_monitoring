import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Control {

    default property alias target: targetContainer.data

    //
    id: root
    padding: 10




    contentItem: Rectangle {

        color: 'red'

        MouseArea {
            id: mouseArea
            anchors.fill: parent
            propagateComposedEvents: true
            hoverEnabled: true
            onPressed: function(mouse) {
                targetContainer.anchors.centerIn = undefined
                targetContainer.x = mouse.x - targetContainer.width/2
                targetContainer.y = mouse.y - targetContainer.height/2
                targetContainer.enabled = true
                console.log('pressed')
                mouse.accepted = false
            }
        }

        Item {

            enabled: mouseArea.containsPress
            opacity: enabled ? 1 : 0.2
            id: targetContainer
            anchors.centerIn: parent
            height: children[0].height
            width: children[0].width
        }
    }

    background: Rectangle {
        color: 'lightgreen'
    }


}

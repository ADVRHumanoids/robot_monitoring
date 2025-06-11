import QtQuick
import QtCore
import QtQuick.Controls

import Main
import Font
import "EmergencyButton.js" as Logic

Item {
    readonly property real margin: 20
    property ClientEndpoint client

    Rectangle {
        id: background
        color: "#eed49f"
        anchors.centerIn: parent
        width: 600
        height: 200
        radius: 10

        DelayButton {
            anchors.centerIn: background
            id: control
            checked: true
            // delay: 100
            width: background.width - margin
            height: background.width - margin



            state: "ready"
            states : [

                State {
                    name: "ready"
                    PropertyChanges {target: icon; source: '/Icons/icons/emergency_stop.png'}
                    PropertyChanges {target: appereance; color:"#ed8796" }
                },

                State {
                    name: "ko"
                    PropertyChanges {target: icon; source: '/Icons/icons/ko.png'}
                    PropertyChanges {target: appereance; color: '#5b6078'}
                    PropertyChanges {target: background; color: '#ee99a0'}
                }
            ]

            background: Rectangle {
                id: appereance
                implicitWidth: background.width - margin
                implicitHeight: background.height - margin
                opacity: enabled ? 1 : 0.3
                color: '#ed8796'
                radius: background.radius

                // readonly property real size: Math.min(width, height)
                width: background.width - margin
                height: background.height - margin
                anchors.centerIn: parent

                Image {
                    id: icon
                    source: "/Icons/icons/emergency_stop.png"
                    anchors.centerIn: parent
                    fillMode: Image.PreserveAspectFit
                    sourceSize {
                        width: 100
                        height: 100
                    }
                }
            }

            onActivated: {
                Logic.kill()
                state = "ko"
            }

            Connections {
                target: client
                onObjectReceived: function(msg) {

                    if (msg.type === 'robot_state') {
                        console.log(msg.state)
                        if (msg.state === 'Alive') {
                            control.state = "ready"
                        }
                        else {
                            control.state = "ko"
                        }
                    }

                    // console.log(msg.type)
                }
            }
        }
    }
}





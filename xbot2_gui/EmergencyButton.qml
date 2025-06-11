import QtQuick
import QtCore
import QtQuick.Controls

import Main
import Font

import "Launcher.js" as Logic

    DelayButton {

        property ClientEndpoint client
        id: control
        checked: true
        delay: 100

        state: "ready"
        states : [

            State {
                name: "ready"
                PropertyChanges {target: icon; source: '/Icons/icons/emergency_stop.png'}
                PropertyChanges {target: appereance; color:"#ed8796" }
            },

            State {
                name: "ko"
                PropertyChanges {target: icon; source: '/Icons/icons/ko.png' }
                PropertyChanges {target: appereance; opacity: 0.69 }
            }

        ]

        contentItem: Image {
            id: icon
            source: '/Icons/icons/emergency_stop.png'
            fillMode: Image.PreserveAspectFit
        }

        background: Rectangle {
            id: appereance
            implicitWidth: 100
            implicitHeight: 100
            opacity: enabled ? 1 : 0.3
            color: '#ed8796'
            radius: size / 2

            readonly property real size: Math.min(control.width, control.height)
            width: size
            height: size
            anchors.centerIn: parent
        }

        onActivated: {
            Logic.pluginCmd("cartesio_imp", 'stop')
            state = 'ko'
        }

        // Connections {
        //     target: client
        //     onObjectReceived: function(msg) {
        //         console.log(msg)
        //     }

        //     function onPluginStatMessageReceived(msg) {
        //         // Logic.onPluginMessageReceived(pluginRepeater, msg)
        //         console.log(msg)
        //     }
        // }
    }





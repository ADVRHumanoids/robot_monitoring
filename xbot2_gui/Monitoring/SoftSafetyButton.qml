import QtQuick
import QtQuick.Controls
import Common
import Main
import Font

import "Monitoring.js" as Logic

DelayButtonRound {

    property ClientEndpoint client

    id: root

    property bool jointActive: true

    visible: CommonProperties.config.showSoftEmergency

    color: jointActive ? 'red' : 'green'

    Label {
        text: MaterialSymbolNames.emergency
        font.family: syms.font.family
        font.pixelSize: 60
        anchors.centerIn: parent
    }

    onActivated: {
        Logic.setSafetyState(!jointActive, client)
        progress = 0
    }

    Connections {

        target: client

        function onObjectReceived(obj) {
            if(obj.type === 'joint_device_info') {
                jointActive = obj.joint_active
            }
        }
    }
}

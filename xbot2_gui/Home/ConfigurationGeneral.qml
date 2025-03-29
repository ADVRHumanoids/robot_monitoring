import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Common

GridLayout {

    columns: 2
    uniformCellHeights: true

    Label {
        text: 'Show Soft Emergency Stop'
    }

    Switch {
        checked: CommonProperties.config.showSoftEmergency
        onClicked: {
            CommonProperties.config.showSoftEmergency = checked
        }
    }

    Label {
        text: 'Show Monitoring Widget'
    }

    Switch {
        checked: CommonProperties.config.showMonWidget
        onClicked: {
            CommonProperties.config.showMonWidget = checked
        }
    }



}

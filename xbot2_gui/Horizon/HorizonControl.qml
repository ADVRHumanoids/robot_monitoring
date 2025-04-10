import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Pane {

    property alias joyXEnabled: chkX.checked
    property alias joyYEnabled: chkY.checked
    property alias maxSpeed: maxSpeedLinearSpinBox.value
    property alias alwaysWalk: enableSwitch.checked
    property alias gaitType: gaitCombo.currentText

    //
    id: root

    contentItem: ColumnLayout {

        Switch {
            visible: true
            id: enableSwitch
            text: 'Enable gait'
        }

        Item {
            visible: false
            width: parent.width
            height: 3
        }

        Label {
            enabled: enableSwitch.checked
            text: ' gait'
            font.pointSize: 10
        }

        ComboBox {
            id: gaitCombo
            enabled: enableSwitch.checked
            Layout.fillWidth: true
            model: ['Trot', 'Walk', 'Crawl']
        }

        Item {
            width: parent.width
            height: 3
        }

        Label {
            text: ' max speed'
            font.pointSize: 10
        }

        DoubleSpinBox1 {
            Layout.fillWidth: true
            width: parent.width
            id: maxSpeedLinearSpinBox
            from: 0.0
            to: 2.0
            decimals: 2
            value: 0.2
        }

        Item {
            width: parent.width
            height: 3
        }

        Label {
            text: ' enabled directions'
            font.pointSize: 10
        }

        Row {
            topPadding: -6
            bottomPadding: -6
            CheckBox {
                id: chkX
                text: 'X'
                checked: true
            }
            CheckBox {
                id: chkY
                text: 'Y'
                checked: false
            }
        }
    }

    background: Rectangle {
        color: Qt.lighter(palette.window)
        opacity: 0.8
        radius: 8
        clip: true
    }

}

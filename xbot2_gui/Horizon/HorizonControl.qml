import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Pane {

    property alias joyXEnabled: chkX.checked
    property alias joyYEnabled: chkY.checked
    property alias maxSpeed: maxSpeedLinearSpinBox.value
    property alias alwaysWalk: enableSwitch.checked

    //
    id: root

    contentItem: ColumnLayout {

        Switch {
            id: enableSwitch
            text: 'Enable'
            onCheckedChanged: {

            }
        }

        Item {
            width: parent.width
            height: 3
        }

        Label {
            text: ' max speed'
            font.pointSize: 10
        }

        DoubleSpinBox {
            Layout.fillWidth: true
            width: parent.width
            id: maxSpeedLinearSpinBox
            from: 0.0
            to: 0.40
            stepSize: 0.05
            value: 0.2
            decimals: 2
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

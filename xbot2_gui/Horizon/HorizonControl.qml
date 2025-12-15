import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Joy

Pane {

    property alias joyXEnabled: chkX.checked
    property alias joyYEnabled: chkY.checked
    property alias maxSpeed: maxSpeedLinearSpinBox.value
    property alias alwaysWalk: enableSwitch.checked
    property alias gaitType: gaitCombo.currentText

    function toggleAcquireGamepad() {
        gamepadCtrlBtn.checked = !gamepadCtrlBtn.checked
    }

    //
    id: root

    property GamepadInterface gamepadIfc: undefined

    Binding {
        target: gamepadIfc
        property: 'enabled'
        value: gamepadCtrlBtn.checked
    }

    Connections {

        target: gamepadIfc.gamepad

        function onButtonUpChanged(value) {
            if(value) {
                maxSpeedLinearSpinBox.increase()
            }
        }

        function onButtonDownChanged(value) {
            if(value) {
                maxSpeedLinearSpinBox.decrease()
            }
        }

        function onButtonSelectChanged(value) {
            if(value) {
                chkY.checked = !chkY.checked
            }
        }
    }

    contentItem: ColumnLayout {

        Button {
            visible: gamepadIfc.connected
            id: gamepadCtrlBtn
            text: `${checked ? 'Disable' : 'Enable'} Gamepad`
            checkable: true
            checked: false
            width: parent.width
        }

        Switch {
            visible: false
            id: enableSwitch
            text: 'Enable gait'
            checked: true
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
            model: ['Trot', 'Crawl']
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
            decimals: 1
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

    Component.onCompleted:  {
        gamepadIfc = CommonProperties.gamepad.registerGamepad('horizon')
    }

}

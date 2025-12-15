import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Main

Control {

    id: root

    GamepadInterface {
        id: gamepadIfc
    }

    property var gamepad: gamepadIfc.gamepad

    contentItem: GridLayout {

        columns: 2

        // Label {
        //     Layout.columnSpan: 2
        //     text: `AVAILABLE GAMEPADS: [${GamepadManager.connectedGamepads}]`
        // }

        Label {
            Layout.columnSpan: 2
            text: `DEVICE ID: ${gamepad.deviceId}`
        }

        Label {
            Layout.columnSpan: 2
            text: `DEVICE NAME: ${gamepad.name}`
        }

        Label {
            Layout.columnSpan: 2
            text: `DEVICE STATE: ${gamepad.connected ? 'Connected' : 'Disconnected'}`
            bottomPadding: 10
        }



        Label {
            text: 'Axis Left X'
        }

        Label {
            text: gamepad.axisLeftX.toFixed(2)
        }

        Label {
            text: 'Axis Left Y'
        }

        Label {
            text: gamepad.axisLeftY.toFixed(2)
        }

        Label {
            text: 'Axis Right X'
        }

        Label {
            text: gamepad.axisRightX.toFixed(2)
        }

        Label {
            text: 'Axis Right Y'
        }

        Label {
            text: gamepad.axisRightY.toFixed(2)
        }

        Label {
            text: 'Left Trigger'
        }

        Label {
            text: gamepad.buttonL2.toFixed(2)
        }

        Label {
            text: 'Right Trigger'
        }

        Label {
            text: gamepad.buttonR2.toFixed(2)
        }


        RowLayout {
            Layout.columnSpan: 2
            uniformCellSizes: true
            ToolButton {
                text: 'A'
                checkable: true
                checked: gamepad.buttonA
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'B'
                checkable: true
                checked: gamepad.buttonB
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'X'
                checkable: true
                checked: gamepad.buttonX
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'Y'
                checkable: true
                checked: gamepad.buttonY
                Layout.fillWidth: true
            }
        }


        RowLayout {
            Layout.columnSpan: 2
            Layout.fillWidth: true
            uniformCellSizes: true
            ToolButton {
                text: 'Left'
                checkable: true
                checked: gamepad.buttonLeft
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'Right'
                checkable: true
                checked: gamepad.buttonRight
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'Up'
                checkable: true
                checked: gamepad.buttonUp
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'Down'
                checkable: true
                checked: gamepad.buttonDown
                Layout.fillWidth: true
            }
        }


        RowLayout {
            Layout.columnSpan: 2
            Layout.fillWidth: true
            uniformCellSizes: true
            ToolButton {
                text: 'Select'
                checkable: true
                checked: gamepad.buttonSelect
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'Start'
                checkable: true
                checked: gamepad.buttonStart
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'LB'
                checkable: true
                checked: gamepad.buttonL1
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'RB'
                checkable: true
                checked: gamepad.buttonR1
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'L1'
                checkable: true
                checked: gamepad.buttonL3
                Layout.fillWidth: true
            }
            ToolButton {
                text: 'R1'
                checkable: true
                checked: gamepad.buttonR3
                Layout.fillWidth: true
            }
        }

    }

}


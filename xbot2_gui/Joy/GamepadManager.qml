import QtQuick
import QtGamepadLegacy as QtGamepad

Item {

    property alias gamepad: gamepad

    Connections {
        target: QtGamepad.GamepadManager
        function onGamepadConnected(deviceId) { gamepad.deviceId = deviceId }
    }

    QtGamepad.Gamepad {
        id: gamepad
        deviceId: QtGamepad.GamepadManager.connectedGamepads.length > 0 ? QtGamepad.GamepadManager.connectedGamepads[0] : -1
    }

}

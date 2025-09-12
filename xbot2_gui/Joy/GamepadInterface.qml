import QtQuick

Item {

    property bool connected: gamepad?.connected ?? false

    property var gamepad: loader.item.gamepad

    id: root

    onEnabledChanged: {
        gamepad.setEnabled(enabled)
        if (enabled) {
            console.log("GamepadInterface enabled")
        } else {
            console.log("GamepadInterface disabled")
        }
    }

    Loader {

        id: loader

        source: appData.hasGamepadCapability ? 'GamepadManager.qml' : 'GamepadManagerDummy.qml'

        onLoaded: {
            gamepad.setEnabled(root.enabled)
            console.log(`gamepad.setEnabled(${root.enabled})`)
        }
    }

    Component.onCompleted: {
        console.log(`appData.hasGamepadCapability: ${appData.hasGamepadCapability}`)
        console.log(`loader.source: ${loader.source}`)
    }

}

import QtQuick 2.15

PadForm {

    id: root

    signal joystickMoved(double x, double y);
    property bool repeat: true
    property real joyX: 0
    property real joyY: 0
    property alias joyPressed: root.mouseArea.containsPress

    onJoystickMoved: function(x, y) {
        joyX = x;
        joyY = y;
    }

    mouseArea.onReleased: {
        returnAnimation.restart()
        joystickMoved(0, 0)
    }

    mouseArea.onPressed: {
        returnAnimation.stop();
    }

    mouseArea.onPositionChanged: {
        if (mouseArea.fingerInBounds) {
            inner.anchors.horizontalCenterOffset = mouseArea.mcx
            inner.anchors.verticalCenterOffset = mouseArea.mcy
        }
        else {
            var angle = Math.atan2(mouseArea.mcy, mouseArea.mcx)
            inner.anchors.horizontalCenterOffset = Math.cos(angle) * mouseArea.distanceBound
            inner.anchors.verticalCenterOffset = Math.sin(angle) * mouseArea.distanceBound
        }

        // Fire the signal to indicate the joystick has moved
        angle = Math.atan2(mouseArea.signal_y, mouseArea.signal_x)

        if(mouseArea.fingerInBounds) {
            joystickMoved(
                        verticalOnly ? 0 : Math.cos(angle) * Math.sqrt(mouseArea.fingerDistance2) / mouseArea.distanceBound,
                        horizontalOnly ? 0 : Math.sin(angle) * Math.sqrt(mouseArea.fingerDistance2) / mouseArea.distanceBound
                        );
        } else {
            joystickMoved(
                        verticalOnly ? 0 : Math.cos(angle) * 1,
                        horizontalOnly ? 0 : Math.sin(angle) * 1
                        );
        }
    }

    Timer {
        repeat: root.repeat
        running: root.joyPressed
        interval: 50
        onTriggered: {
            joystickMoved(joyX, joyY)
        }
        onRunningChanged: {
            if(!root.joyPressed)
            {
                joystickMoved(0, 0)
            }
        }
    }
}

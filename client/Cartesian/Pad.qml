import QtQuick 2.15

PadForm {

    id: root

    property int side: side
    property alias verticalOnly : root.verticalOnly
    property alias horizontalOnly : root.horizontalOnly
    signal joystickMoved(double x, double y);

    mouseArea.onReleased: {
        returnAnimation.restart()
    }

    mouseArea.onPressed: {
        returnAnimation.stop();
        joystickMoved(0, 0)
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
}

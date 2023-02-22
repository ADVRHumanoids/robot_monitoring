import QtQuick
import QtQuick.Controls

Item {

    property alias mouseX: tp.x
    property alias mouseY: tp.y
    property alias containsPress: tp.pressed
    signal pressed()
    signal released()
    signal positionChanged()

    id: root

    MultiPointTouchArea {
        anchors.fill: parent
        touchPoints: TouchPoint {
            id: tp
            onXChanged: Qt.callLater(root.positionChanged)
            onYChanged: Qt.callLater(root.positionChanged)
            onPressedChanged: if(tp.pressed) root.pressed(); else root.released()
        }
    }

}

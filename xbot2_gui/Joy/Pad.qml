import QtQuick
import QtQuick.Controls
import Common

Item {


    signal joystickMoved(double x, double y);
    property bool repeat: true
    property real joyX: 0
    property real joyY: 0
    property alias joyPressed: mouseArea.containsPress
    property bool active

    //
    id: root

    property int side: 200
    property alias mouseArea: mouseArea
    property alias inner: inner
    property alias returnAnimation: returnAnimation
    property bool verticalOnly : false
    property bool horizontalOnly : false
    property string xLabel: 'Y'
    property string yLabel: 'X'
    property alias color: outer.color

    implicitWidth: 200
    implicitHeight: 200

    Rectangle {

        id: area
        anchors.fill: parent
        color: Qt.rgba(1, 1, 1, 0.2)
        radius: 12

        MultiMouseArea {
            id: mouseArea
            anchors.fill: parent
            z: 1

            onPressed: {
                returnAnimation.stop();
                outer.animatePosChange = false
                root.active = true
                offsetX = mouseArea.mouseX
                offsetY = mouseArea.mouseY
                outer.x = mouseArea.mouseX - outer.width/2
                outer.y = mouseArea.mouseY - outer.height/2
            }

            onReleased: {
                outer.animatePosChange = true
                root.active = false
//                outer.x = (area.width - outer.width) / 2
//                outer.y = (area.height - outer.height) / 2
                returnAnimation.restart()
                joystickMoved(0, 0)
            }

            property real offsetX
            property real offsetY
            property real mouseX2 : verticalOnly ? offsetX : mouseX
            property real mouseY2 : horizontalOnly ? offsetY : mouseY
            property real fingerAngle : Math.atan2(mouseX2, mouseY2)
            property int mcx : mouseX2 - offsetX
            property int mcy : mouseY2 - offsetY
            property bool fingerInBounds : fingerDistance2 < distanceBound2
            property real fingerDistance2 : mcx * mcx + mcy * mcy
            property real distanceBound : root.side * 0.5 - inner.width * 0.5
            property real distanceBound2 : distanceBound * distanceBound

            property double signal_x : (mouseX2 - outer.width/2) / distanceBound
            property double signal_y : -(mouseY2 - outer.height/2) / distanceBound

        }

        Rectangle {

            property bool animatePosChange: false

            id: outer
            x: (area.width - outer.width) / 2
            y: (area.height - outer.height) / 2
            width: verticalOnly ? root.side/4. + 10 : root.side
            height: horizontalOnly ? root.side/4. + 10 : root.side
            radius: root.side/2.
            color: CommonProperties.colors.primary
            opacity: root.active ? 1 : 0.1


            Behavior on x {
                enabled: outer.animatePosChange
                NumberAnimation {
                    easing.type: Easing.OutQuad
                    duration: 200
                }
            }

            Behavior on y {
                enabled: outer.animatePosChange
                NumberAnimation {
                    easing.type: Easing.OutQuad
                    duration: 200
                }
            }

            Behavior on opacity {
                NumberAnimation {
                    easing.type: Easing.OutQuad
                    duration: 200
                }
            }

            Rectangle {

                id: inner
                anchors.centerIn: parent
                width: root.side/4.
                height: width
                radius: width/2.
                color: CommonProperties.colors.accent

                MouseArea {
                    anchors.fill: parent
                    hoverEnabled: true
                }
            }

            Label {
                text: '-' + xLabel
                anchors.right: parent.left
                anchors.verticalCenter: parent.verticalCenter
                anchors.margins: 5
                visible: !verticalOnly
            }

            Label {
                text: '+' + xLabel
                anchors.left: parent.right
                anchors.verticalCenter: parent.verticalCenter
                anchors.margins: 5
                visible: !verticalOnly
            }

            Label {
                text: '-' + yLabel
                anchors.top: parent.bottom
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.margins: 5
                visible: !horizontalOnly
            }

            Label {
                text: '+' + yLabel
                anchors.bottom: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.margins: 5
                visible: !horizontalOnly
            }

            ParallelAnimation {

                id: returnAnimation

                NumberAnimation {
                    target: inner.anchors;
                    property: "horizontalCenterOffset";
                    to: 0;
                    duration: 200;
                    easing.type: Easing.OutSine
                }

                NumberAnimation {
                    target: inner.anchors;
                    property: "verticalCenterOffset";
                    to: 0;
                    duration: 200;
                    easing.type: Easing.OutSine
                }

            }

        }

    }

    onJoystickMoved: function(x, y) {
        joyX = x;
        joyY = y;
    }

    mouseArea.onPositionChanged: Qt.callLater(function(){

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

        if(mouseArea.fingerInBounds && mouseArea.containsPress) {
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
    })

    Timer {
        repeat: root.repeat
        running: root.joyPressed
        interval: 50
        onTriggered: {
            root.joystickMoved(joyX, joyY)
        }
    }

    Timer {
        running: !root.joyPressed
        interval: 20
        onTriggered: {
            joyX = 0
            joyY = 0
            root.joystickMoved(0, 0)
        }
    }
}

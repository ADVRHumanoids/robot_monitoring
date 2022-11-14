import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

Item {

    id: rect

    property alias side: rect.width
    property alias mouseArea: mouseArea
    property alias inner: inner
    property alias returnAnimation: returnAnimation
    property bool verticalOnly : false
    property bool horizontalOnly : false
    property string xLabel: 'Y'
    property string yLabel: 'X'
    property alias color: outer.color

    width: 200
    height: width

    Rectangle {

        id: outer
        anchors.centerIn: parent
        width: verticalOnly ? rect.side/4. + 10 : rect.side
        height: horizontalOnly ? rect.side/4. + 10 : rect.side
        radius: rect.side/2.
        color: Material.accent

        Rectangle {

            id: inner
            anchors.centerIn: parent
            width: rect.side/4.
            height: width
            radius: width/2.
            color: Qt.rgba(0, 0, 0, 0.4)

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

        MouseArea {
            id: mouseArea
            anchors.fill: parent
            z: 1

            property real mouseX2 : verticalOnly ? width * 0.5 : mouseX
            property real mouseY2 : horizontalOnly ? height * 0.5 : mouseY
            property real fingerAngle : Math.atan2(mouseX2, mouseY2)
            property int mcx : mouseX2 - width * 0.5
            property int mcy : mouseY2 - height * 0.5
            property bool fingerInBounds : fingerDistance2 < distanceBound2
            property real fingerDistance2 : mcx * mcx + mcy * mcy
            property real distanceBound : rect.side * 0.5 - inner.width * 0.5
            property real distanceBound2 : distanceBound * distanceBound

            property double signal_x : (mouseX2 - outer.width/2) / distanceBound
            property double signal_y : -(mouseY2 - outer.height/2) / distanceBound
        }

        ParallelAnimation {
            id: returnAnimation
            NumberAnimation { target: inner.anchors; property: "horizontalCenterOffset";
                to: 0; duration: 200; easing.type: Easing.OutSine }
            NumberAnimation { target: inner.anchors; property: "verticalCenterOffset";
                to: 0; duration: 200; easing.type: Easing.OutSine }
        }

    }

}

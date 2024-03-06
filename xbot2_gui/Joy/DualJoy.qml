import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Main
import Video
import "../Video/VideoStream.js" as VideoStream
import "Joy.js" as Joy
import Common
import Font

Item {

    property bool compactLayout: false

    property int padSide: 200

    property alias leftPad: leftPad

    property alias rightPad: rightPad

    property bool joyPressed: leftPad.joyPressed || rightPad.joyPressed

    signal leftPadMoved(real x, real y)

    signal rightPadMoved(real x, real y)

    id: root

    RowLayout {

        visible: !root.compactLayout

        anchors {
            fill: parent
        }

        LayoutItemProxy {
            target: leftPad
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: parent.width / 2
        }

        Item {
            Layout.fillWidth: true
            Layout.minimumWidth: 50
        }

        LayoutItemProxy {
            target: rightPad
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: parent.width / 2
        }

    }

    RowLayout {

        visible: root.compactLayout

        anchors {
            left: parent.left
            bottom: parent.bottom
            right: parent.right
            top: parent.top
            margins: 16
        }

        TabButton {
            text: MaterialSymbolNames.arrowBack
            font.family: MaterialSymbolNames.filledFont.font.family
            onClicked: padSwipe.decrementCurrentIndex()
            font.pointSize: 20
            z: -1
            enabled: padSwipe.currentIndex > 0
        }

        SwipeView {

            id: padSwipe

            interactive: false

            Layout.fillWidth: true
            Layout.fillHeight: true

            LayoutItemProxy {
                target: leftPad
                visible: padSwipe.currentIndex === 0
            }

            LayoutItemProxy {
                target: rightPad
                visible: padSwipe.currentIndex === 1
            }

        }

        TabButton {
            text: MaterialSymbolNames.arrowForward
            font.family: MaterialSymbolNames.filledFont.font.family
            font.pointSize: 20
            onClicked: padSwipe.incrementCurrentIndex()
            z: -1
            enabled: padSwipe.currentIndex + 1 < padSwipe.count
        }

    }

    Pad  {

        id: leftPad

        side: root.padSide

        backgroundColor: 'transparent'

        onJoystickMoved: (x, y) => root.leftPadMoved(x, y)

    }

    Pad  {

        id: rightPad

        side: root.padSide

        backgroundColor: 'transparent'

        onJoystickMoved: (x, y) => root.rightPadMoved(x, y)
    }

}

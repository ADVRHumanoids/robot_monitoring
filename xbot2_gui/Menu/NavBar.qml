import QtQuick
import QtQuick.Controls
import Qt5Compat.GraphicalEffects

import Common

TabBar {
    id: root
    // Relying on the left/right padding to center teh content, so do not include it in the implicit
    // size.
    implicitWidth: Math.max(implicitBackgroundWidth + leftInset + rightInset, contentWidth)
    implicitHeight: Math.max(implicitBackgroundHeight + topInset + bottomInset, contentHeight)
    spacing: 1
    topPadding: (height - contentHeight) / 2
    leftPadding: (width - contentWidth) / 2
    rightPadding: (width - contentWidth) / 2
    bottomPadding: (height - contentHeight) / 2

    contentWidth: Math.min(width - 2*10, contentItem.implicitWidth)

    contentItem: Item {
        implicitWidth: list.contentItem.width
        implicitHeight: list.contentItem.height

//        DebugRectangle {
//            target: parent
//        }

        WheelHandler {
            onWheel: (event) => {
                         list.flick(event.angleDelta.y*event.y*0.2, 0)
                     }
        }

        ListView {
            id: list
            anchors.fill: parent
            model: root.contentModel
            currentIndex: root.currentIndex
            spacing: root.spacing
            orientation: Qt.Horizontal
            boundsBehavior: Flickable.StopAtBounds
            flickableDirection: Flickable.HorizontalFlick
            snapMode: ListView.SnapToItem
            highlightMoveDuration: 0
            highlightRangeMode: ListView.ApplyRange
            preferredHighlightBegin: 0
            preferredHighlightEnd: width
            clip: true
        }
    }

    background: Rectangle {
        implicitWidth: 400
        implicitHeight: 60
        color: root.palette.base
        radius: 12

    }
}

import QtQuick
import QtQuick.Controls
import Qt5Compat.GraphicalEffects

import Common

// VertTabBar.qml
Container {
    id: control
    implicitHeight: list.contentHeight
    property alias listWidth: list.width

    contentItem: ListView {
        id: list
        model: control.contentModel
        currentIndex: control.currentIndex

        spacing: control.spacing
        orientation: ListView.Vertical   // <<-- VERTICAL
        boundsBehavior: Flickable.StopAtBounds
        flickableDirection: Flickable.AutoFlickIfNeeded
        snapMode: ListView.SnapToItem

        highlightMoveDuration: 0
        highlightRangeMode: ListView.ApplyRange
        preferredHighlightBegin: 40
        preferredHighlightEnd: height - 40
    }

}

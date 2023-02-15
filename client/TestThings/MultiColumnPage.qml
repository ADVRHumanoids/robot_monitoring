import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import xbot2_gui.common

Item {

    // public

    property list<Item> columnItems
    property int mobileBreakpoint: 400


    // private

    id: root

    property bool mobileLayout: width < CommonProperties.geom.mobileBreakpoint

    function _responsiveLayout() {
        if(mobileLayout) {
            row.children = []
            swipe.contentChildren = columnItems
        }
        else {
            swipe.contentChildren = []
            row.children = columnItems
        }
    }

    onMobileLayoutChanged: {
        root._responsiveLayout()
    }

    Component.onCompleted: {
        root._responsiveLayout()
    }

    TabBar {
        id: bar
        width: parent.width

        Repeater {
            model: root.columnItems
            TabButton {
                text: modelData.objectName
            }
        }

        y: mobileLayout ? 0 : -height
    }

    SwipeView {
        id: swipe
        clip: true
        anchors {
            top: bar.bottom
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            margins: 16
        }
        interactive: false
        currentIndex: bar.currentIndex
    }

    RowLayout {
        id: row
        spacing: 16

        anchors {
            top: bar.bottom
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            margins: 16
        }
    }
}

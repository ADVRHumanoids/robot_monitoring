import QtQuick
import QtQuick.Controls
import xbot2_gui.common

Rectangle {
    color: CommonProperties.colors.cardBackground
    radius: CommonProperties.geom.cardRadius
    property alias headerText: label.text
    property alias headerPixelSize: label.font.pixelSize
    property Item toolbar: Rectangle {
        color: 'blue'
    }
    property real topInset: 16
    property real bottomInset: 16
    property real leftInset: 16
    property real rightInset: 16

    id: root
    implicitHeight: topInset +
                    label.implicitHeight + toolbarContainer.implicitHeight + contentItem.implicitHeight +
                    bottomInset + 16
    implicitWidth: Math.max(label.implicitWidth + 16 + toolbarContainer.implicitWidth,
                            contentItem.implicitWidth) + leftInset + rightInset

    // header
    Label {
        id: label
        font.pixelSize: CommonProperties.font.h1
        anchors {
            top: parent.top
            left: parent.left
            margins: 16
        }
    }

    // toolbar area
    Item {
        id: toolbarContainer
//        implicitHeight: childrenRect.height + topInset + bottomInset
//        implicitWidth: childrenRect.width + leftInset + rightInset
        anchors {
            top: label.top; bottom: label.bottom; left: label.right; right: parent.right
            leftMargin: 16
            rightMargin: 16
        }
    }

    // content
    Item {
        id: contentItem
        implicitHeight: 0
        implicitWidth: 0
        anchors {
            top: label.bottom
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            margins: 16
        }
    }

    // layout logic
    Component.onCompleted: {

        //  position toolbar
        toolbar.parent = toolbarContainer
        toolbar.anchors.fill = toolbarContainer

        // position children
        let contentChild = children[3]
        contentItem.implicitHeight = contentChild.implicitHeight
        contentItem.implicitWidth = contentChild.implicitWidth
        contentChild.parent = contentItem

        if(children.length > 4) {
            console.log('You should only set one item as child of Card; it will re-parented to contentItem')
        }

    }
}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Item {

    property var model

    property int currentIndex: 0

    signal hamburgerClicked()

    property int orientation: Qt.Vertical

    property int checkedDisplayMode: AbstractButton.TextUnderIcon

    property int uncheckedDisplayMode: AbstractButton.TextUnderIcon

    property list<Item> buttonItems

    function setBadgeNumber(i, num) {
        buttonItems[i].badgeNum = num
    }

    //
    id: root

    property Item activePositioner: orientation === Qt.Vertical ? col : row

    implicitHeight: activePositioner.implicitHeight
    implicitWidth: activePositioner.implicitWidth

    property Component barButtonComponent: Component {
        Item {
            width: btn.width
            height: btn.height

            property string iconChar
            property string text
            property int index
            property alias badgeNum: btn.badgeNum

            NavButton {
                id: btn
                checkedDisplayMode: root.checkedDisplayMode
                uncheckedDisplayMode: root.uncheckedDisplayMode
                textFixedWidth: 50
                anchors.centerIn: parent
                iconChar: parent.iconChar
                text: parent.text
                checked: root.currentIndex === parent.index
                onClicked: root.currentIndex = parent.index
            }
        }
    }

    Component.onCompleted: {
        if(model === undefined) {
            return
        }

        for(let i = 0; i < model.children.length; i++) {
            let obj = model.children[i]
            if(obj instanceof Repeater) continue
            let btn = barButtonComponent.createObject(activePositioner,
                                                      {'text': obj.name, 'iconChar': obj.iconText, 'index': i, 'enabled': Qt.binding(() => { return true || obj.active })})
            buttonItems.push(btn)
        }
    }

    Flickable {
        id: vScroll
        width: parent.width
        height: Math.min(contentHeight, parent.height - 100)
        anchors.centerIn: visible ? parent : undefined
        contentWidth: width
        contentHeight: col.height
        visible: root.orientation === Qt.Vertical
        clip: true

        Column {
            id: col
            width: vScroll.width
            spacing: 5
        }
    }

    MouseArea {

        z: 1

        anchors.fill: hScroll

        enabled: hScroll.visible

        acceptedButtons: Qt.NoButton

        onWheel: function(wheel) {
            hScroll.flick(wheel.angleDelta.y * 10, 0)
        }
    }

    Flickable {
        id: hScroll
        width: Math.min(contentWidth, parent.width - 100)
        height: parent.height
        anchors.centerIn: visible ? parent : undefined
        contentHeight: height
        contentWidth: row.width
        visible: root.orientation === Qt.Horizontal
        clip: true

        Row {
            id: row
            height: hScroll.height
            spacing: 5
        }
    }

}


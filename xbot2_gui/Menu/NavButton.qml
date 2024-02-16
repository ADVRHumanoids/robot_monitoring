import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Font
import Common

TabButton {

    required property string iconChar
    property int checkedDisplayMode: TabButton.TextBesideIcon
    property int uncheckedDisplayMode: TabButton.IconOnly
    property int textFixedWidth: -1
    property alias badgeNum: badge.num
    property real sizeFactor: 1.0

    //
    id: root
    width: implicitWidth
    height: implicitHeight
    display: checked ? checkedDisplayMode : uncheckedDisplayMode
    topPadding: 1 //10
    leftPadding: display === AbstractButton.IconOnly ? 1 : 14 // 14
    rightPadding: display === AbstractButton.IconOnly ? 1 : 14 // 14
    bottomPadding: 1 //10
    spacing: 10
    clip: true
    icon.color: checked ? palette.highlightedText : palette.buttonText
    font.bold: checked
    font.pointSize: Qt.application.font.pointSize * 0.85
    opacity: enabled ? 1 : 0.5

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true
        acceptedButtons: Qt.NoButton
        enabled: parent.enabled
    }

    contentItem: Item {
        implicitWidth: visibleItem.implicitWidth
        implicitHeight: visibleItem.implicitHeight

        property Item visibleItem: row.visible ? row : (col.visible ? col: iconOnly)

        Row {
            id: row
            visible: root.display === TabButton.TextBesideIcon
            spacing: root.spacing
            LayoutItemProxy {
                target: img
                anchors.verticalCenter: parent === undefined ? undefined : parent.verticalCenter
            }
            LayoutItemProxy {
                target: labelWrapper
                anchors.verticalCenter: parent === undefined ? undefined : parent.verticalCenter
            }
        }

        Column {
            id: col
            visible: root.display === TabButton.TextUnderIcon
            spacing: 1
            LayoutItemProxy {
                target: img
                anchors.horizontalCenter: parent === undefined ? undefined : parent.horizontalCenter
            }
            LayoutItemProxy {
                target: labelWrapper
                anchors.horizontalCenter: parent === undefined ? undefined : parent.horizontalCenter
            }
        }

        LayoutItemProxy {
            id: iconOnly
            visible: root.display === TabButton.IconOnly
            target: img
        }

        Item {
            id: img
            visible: root.display !== TabButton.TextOnly

            implicitHeight: imgTxt.visible ? imgTxt.implicitHeight - 6 : imgPng.height
            implicitWidth: imgTxt.visible ? imgTxt.implicitWidth : imgPng.width
            height: imgTxt.visible ? imgTxt.height - 10 : imgPng.height
            width: imgTxt.visible ? imgTxt.width : imgPng.width

            Text {
                id: imgTxt
                text: root.iconChar
                color: root.icon.color
                font.family: materialSymbols.font.family
                font.pointSize: root.icon.height * root.sizeFactor
                padding: 0
                visible: !imgPng.visible

                MaterialSymbols {
                    id: materialSymbols
                    filled: root.checked
                }

                Rectangle {
                    id: badge
                    property int num: 0
                    color: 'red'
                    visible: num > 0
                    width: Math.max(badgeLabel.width, badgeLabel.height)
                    height: badgeLabel.height
                    radius: height/2
                    x: parent.width - width
                    y: 0
                    Label {
                        id: badgeLabel
                        text: badge.num < 99 ? badge.num : '99+'
                        anchors.centerIn: parent
                        padding: 2
                    }
                }

            }

            Item {
                id: imgPng
                height: imgTxt.height * 0.9
                width: imgTxt.width * 0.9
                visible: root.icon.source !== Qt.url('')
                Image {
                    anchors.centerIn: parent
                    source: root.icon.source
                    height: parent.height
                    width: parent.height
                    antialiasing: true
                    smooth: true
//                    DebugRectangle {
//                        target: parent
//                    }
                }
            }

        }

        Item {
            id: labelWrapper
            implicitHeight: label.implicitHeight
            implicitWidth: (root.textFixedWidth > 0 && root.display === AbstractButton.TextUnderIcon) ? root.textFixedWidth : label.implicitWidth

            Label {
                id: label
                text: root.text
                font: root.font
                color: root.icon.color
                visible: root.display !== TabButton.IconOnly
                anchors.centerIn: parent
            }

        }
    }

    background: Item {
        //        implicitWidth: 30
        //        implicitHeight: 30

        Rectangle {
            anchors.fill: parent
            color: root.palette.active.highlight
            radius: parent.height * 0.6
            opacity: root.checked ? 1 : (mouseArea.containsMouse ? 0.2 : 0)
            //            visible: root.checked || mouseArea.containsMouse
            Behavior on opacity {
                NumberAnimation {}
            }
        }


    }

    Behavior on width {
        NumberAnimation {
            duration: 250
            easing.type: Easing.OutBack
        }
    }

    Behavior on height {
        NumberAnimation {
            duration: 250
            easing.type: Easing.OutBack
        }
    }
}

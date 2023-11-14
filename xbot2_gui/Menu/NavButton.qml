import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Font
import Common

TabButton {

    required property string iconChar
    property int checkedDisplayMode: TabButton.TextBesideIcon
    property int uncheckedDisplayMode: TabButton.IconOnly

    //
    id: root
    width: implicitWidth
    height: implicitHeight
    display: checked ? checkedDisplayMode : uncheckedDisplayMode
    topPadding: 10
    leftPadding: 14
    rightPadding: 14
    bottomPadding: 10
    spacing: 10
    clip: true
    icon.color: checked ? palette.highlightedText : palette.buttonText
    font.bold: checked

    palette {
        base: CommonProperties.colors.primary
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
                target: label
                anchors.verticalCenter: parent === undefined ? undefined : parent.verticalCenter
            }
        }

        Column {
            id: col
            visible: root.display === TabButton.TextUnderIcon
            spacing: root.spacing * 0.5
            LayoutItemProxy {
                target: img
                anchors.horizontalCenter: parent === undefined ? undefined : parent.horizontalCenter
            }
            LayoutItemProxy {
                target: label
                anchors.horizontalCenter: parent === undefined ? undefined : parent.horizontalCenter
            }
        }

        LayoutItemProxy {
            id: iconOnly
            visible: root.display === TabButton.IconOnly
            target: img
        }

        Text {
            id: img
            text: root.iconChar
            visible: root.display !== TabButton.TextOnly
            color: root.icon.color
            font.family: materialSymbols.font.family
            font.pointSize: root.icon.height
            padding: -10
            MaterialSymbols {
                id: materialSymbols
                filled: root.checked
            }

        }

        Label {
            id: label
            text: root.text
            font: root.font
            color: root.checked ? root.palette.highlightedText : root.palette.buttonText
            visible: root.display !== TabButton.IconOnly
        }
    }
    background: Item {
        implicitWidth: 30
        implicitHeight: 30

        Rectangle {
            anchors.fill: parent
            color: root.palette.highlight
            radius: parent.height * 0.6
            visible: root.checked
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

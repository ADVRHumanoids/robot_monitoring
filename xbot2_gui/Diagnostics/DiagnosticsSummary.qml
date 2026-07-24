import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

import "Diagnostics.js" as Logic

Control {

    signal focusActiveIssue(string path)

    id: root

    property alias model: repeater.model

    contentItem: ColumnLayout {

        spacing: 12

        RowLayout {

            Layout.fillWidth: true

            Label {
                text: 'Active issues'
                font.capitalization: Font.AllUppercase
                font.bold: true
                color: palette.accent
                font.pixelSize: CommonProperties.font.h3
                Layout.fillWidth: true
            }

            ToolButton {
                id: warnBtn
                text: 'Warning'
                checkable: true
                checked: true
            }

            ToolButton {
                id: staleBtn
                text: 'Stale'
                checkable: true
                checked: true
            }

        }

        ScrollView {
            id: scroll
            Layout.fillWidth: true
            Layout.fillHeight: true
            contentWidth: availableWidth

            GridLayout {
                width: scroll.contentWidth
                columns: Math.ceil(width / 400)
                rows: Math.ceil(repeater.count / columns)
                uniformCellHeights: true
                uniformCellWidths: true
                columnSpacing: 16
                rowSpacing: 16
                Label {
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    text: 'There are no active issues'
                    color: palette.disabled.text
                    font.pixelSize: CommonProperties.font.h3
                    visible: repeater.count === 0
                }

                Repeater {
                    id: repeater
                    delegate: summaryDelegate
                }
            }
        }
    }

    property Component summaryDelegate: Component {
        ItemDelegate {
            visible: modelData.level === 2 ||
                     (modelData.level === 1 && warnBtn.checked) ||
                     (modelData.level === 3 && staleBtn.checked)
            Layout.fillHeight: true
            Layout.fillWidth: true
            padding: 12
            background: Rectangle {
                color: palette.base
                radius: 4
            }
            onClicked: root.focusActiveIssue(modelData.path)
            contentItem: ColumnLayout {

                spacing: 12

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 12
                    Rectangle {
                        Layout.preferredHeight: pathLabel.height
                        Layout.minimumWidth: height
                        Layout.preferredWidth: height
                        radius: height / 2
                        color: Logic.levelToColor(modelData.level)
                    }
                    Label {
                        id: pathLabel
                        Layout.fillWidth: true
                        text: modelData.path
                        font.bold: true
                        elide: Text.ElideLeft
                    }
                }

                Label {
                    Layout.fillWidth: true
                    text: modelData.message
                    wrapMode: Text.Wrap
                    font.pixelSize: 10
                }

            }
        }
    }

}

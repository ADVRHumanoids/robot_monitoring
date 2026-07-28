import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

import "Diagnostics.js" as Logic

Control {

    signal focusActiveIssue(string path)

    id: root

    property var model

    contentItem: GridLayout {

        columns: 2
        rows: 1

        columnSpacing: 12

        IssueListView {
            Layout.fillHeight: true
            Layout.fillWidth: true
            model: root.model
            title: 'Errors'
            level: 2
        }

        IssueListView {
            Layout.fillHeight: true
            Layout.fillWidth: true
            model: root.model
            title: 'Warnings'
            level: 1
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

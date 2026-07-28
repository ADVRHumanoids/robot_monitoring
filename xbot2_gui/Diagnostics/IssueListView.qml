import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "Diagnostics.js" as Logic

import Common

Control {

    property int level
    property string title
    property alias model: listView.model

    //
    id: root

    contentItem: ListView {
        id: listView

        header: Component {
            Control {
                padding: 12
                topPadding: 6
                bottomPadding: 6
                width: listView.width
                contentItem: RowLayout {
                    Label {
                        id: titleLabel
                        Layout.fillWidth: true
                        text: root.title
                        font.pixelSize: CommonProperties.font.h3
                        font.bold: true
                        font.capitalization: Font.AllUppercase
                        color: Logic.levelToColor(root.level)
                    }
                    ToolButton {
                        id: staleBtn
                        text: 'Show stale'
                        Layout.preferredHeight: titleLabel.height
                    }
                    Label {
                        text: listView.count
                        Layout.preferredWidth: height
                        font.pixelSize: CommonProperties.font.h3
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        padding: 4
                        color: Qt.lighter(Logic.levelToColor(root.level), 1.5)
                        background: Rectangle {
                            radius: height / 2
                            color: Logic.levelToColor(root.level)
                        }
                    }
                }
                background: Rectangle {
                    color: Qt.darker(Logic.levelToColor(root.level))
                    border.color: Qt.darker(Logic.levelToColor(root.level), 3.0)
                }
            }
        }

        delegate: ItemDelegate {
            visible: modelData.level === root.level ||
                     (modelData.level === 3 && staleBtn.checked)
            width: listView.width
            padding: 6
            background: Rectangle {
                color: palette.base
                border.color: Qt.rgba(1, 1, 1, 0.2)
            }
            onClicked: root.focusActiveIssue(modelData.path)
            contentItem: ColumnLayout {

                spacing: 6

                Label {
                    id: pathLabel
                    Layout.fillWidth: true
                    text: modelData.path
                    font.bold: true
                    elide: Text.ElideLeft
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

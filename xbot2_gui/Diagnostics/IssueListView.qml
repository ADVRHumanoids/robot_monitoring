import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "Diagnostics.js" as Logic

import Common

Control {

    property int level
    property string title
    property var model
    property bool showStale: false

    signal focusActiveIssue(string path)

    //
    id: root
    property int count: -1
    property int countStale: -1
    property bool showStaleItems: showStale
    property var filteredModel: []
    property bool enableRefresh: true

    function updateFilteredModel() {
        if(!enableRefresh)
        {
            return
        }

        const filteredIssues = []
        count = 0
        countStale = 0
        if (!model) {
            filteredModel = filteredIssues
            return
        }

        for (let i = 0; i < model.length; ++i) {
            const issue = model[i]
            const matchesLevel = issue.level === level
            const isStale = issue.level === 3
            count += matchesLevel ? 1 : 0
            countStale += isStale ? 1 : 0

            if (matchesLevel || (isStale && showStaleItems))
                filteredIssues.push(issue)
        }

        filteredModel = filteredIssues
    }

    onModelChanged: updateFilteredModel()
    onLevelChanged: updateFilteredModel()
    onShowStaleItemsChanged: updateFilteredModel()
    Component.onCompleted: updateFilteredModel()

    background: Rectangle {
        color: Qt.alpha(palette.base, 0.333)

        Label {
            visible: root.count === 0 && root.countStale === 0
            anchors.centerIn: parent
            font.pixelSize: CommonProperties.font.h2
            text: `No ${root.title.toLowerCase()} found`
            horizontalAlignment: Text.AlignHCenter
        }
    }

    contentItem: ListView {
        id: listView
        model: root.filteredModel
        clip: true
        headerPositioning: ListView.OverlayHeader
        spacing: 3

        onMovementStarted: root.enableRefresh = false

        header: Component {
            Control {
                z: 10
                padding: 12
                topPadding: 6
                bottomPadding: 6
                width: listView.width
                contentItem: RowLayout {
                    spacing: 8
                    // title
                    Label {
                        id: titleLabel
                        Layout.fillWidth: true
                        text: root.title
                        font.pixelSize: CommonProperties.font.h3
                        font.bold: true
                        font.capitalization: Font.AllUppercase
                        color: Logic.levelToColor(root.level)
                    }
                    // refresh
                    ToolButton {
                        id: refreshBtn
                        text: 'Refresh'
                        Layout.preferredHeight: titleLabel.height
                        checkable: true
                        checked: true
                        onClicked: root.enableRefresh = checked
                        Connections {
                            target: root
                            function onEnableRefreshChanged() {
                                refreshBtn.checked = root.enableRefresh
                            }
                        }
                    }
                    // show stale btn
                    ToolButton {
                        id: staleBtn
                        text: 'Show stale'
                        Layout.preferredHeight: titleLabel.height
                        visible: root.showStale
                        checkable: true
                        checked: root.showStaleItems
                        onToggled: root.showStaleItems = checked
                    }
                    // count
                    Label {
                        text: root.count
                        font.pixelSize: CommonProperties.font.h3
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        padding: 4
                        leftPadding: 8
                        rightPadding: 8
                        color: Qt.darker(Logic.levelToColor(root.level), 2.0)
                        background: Rectangle {
                            radius: height / 2
                            color: Logic.levelToColor(root.level)
                        }
                    }
                    // stale count
                    Label {
                        text: root.countStale
                        visible: root.showStale
                        font.pixelSize: CommonProperties.font.h3
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        padding: 4
                        leftPadding: 8
                        rightPadding: 8
                        color: Qt.darker(Logic.levelToColor(3), 2.0)
                        background: Rectangle {
                            radius: height / 2
                            color: Logic.levelToColor(3)
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
            width: listView.width
            padding: 6
            background: Rectangle {
                color: palette.base
                border.color: Qt.rgba(1, 1, 1, 0.2)
                radius: 6
            }
            onClicked: root.focusActiveIssue(modelData.path)
            contentItem: GridLayout {

                columnSpacing: 6
                rowSpacing: 6
                rows: 2
                columns: 2

                // badge rectangle
                Rectangle {
                    radius: 6
                    Layout.rowSpan: 2
                    Layout.fillHeight: true
                    Layout.preferredWidth: 12
                    color: Logic.levelToColor(modelData.level)
                }

                // path
                Label {
                    id: pathLabel
                    Layout.fillWidth: true
                    text: modelData.path
                    font.bold: true
                    elide: Text.ElideLeft
                }

                // message
                Label {
                    Layout.fillWidth: true
                    text: modelData.message
                    wrapMode: Text.Wrap
                    font.pixelSize: 11
                }

            }

        }

    }

}

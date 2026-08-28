import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "Diagnostics.js" as Logic
import Common
import Font

ColumnLayout {

    property alias selectionModel: treeView.selectionModel
    property alias model: treeView.model
    property alias fullModel: treeModel

    function loadData(msg) {
        Logic.onObjectReceived(msg)
    }

    function focusItem(path, resetFilters) {
        Logic.focusItem(path, resetFilters)
    }

    //
    id: root
    spacing: 12



    RowLayout {

        Layout.fillWidth: true

        // Label {
        //     text: 'Diagnostics explorer'
        //     font.capitalization: Font.AllUppercase
        //     font.bold: true
        //     color: palette.accent
        //     font.pixelSize: CommonProperties.font.h3
        // }

        SearchField {
            id: searchField
            Layout.fillWidth: true
            suggestionModel: treeModel
            textRole: 'path'
        }


        Item {
            Layout.preferredWidth: 6
        }

        SmallToolButton {
            id: collapseButton
            text: MaterialSymbolNames.collapseAll
            font.family: 'Material Symbols Outlined'
            font.variableAxes: {'opsz': 48}
            font.pixelSize: 16
            onClicked: Logic.collapseAllVisible()
        }
        SmallToolButton {
            id: expandButton
            text: MaterialSymbolNames.expandAll
            font.family: 'Material Symbols Outlined'
            font.variableAxes: {'opsz': 48}
            font.pixelSize: 16
            onClicked: Logic.expandAllVisible()
        }
        ToolButton {
            id: infoButton
            text: 'Info'
            property bool filterIncludesLevel: true
            checkable: true
            checked: filterIncludesLevel
            onClicked: {
                filterModel.enableLevel(0, !filterIncludesLevel)
            }
        }
        ToolButton {
            id: warningButton
            text: 'Warnings'
            property bool filterIncludesLevel: true
            checkable: true
            checked: filterIncludesLevel
            onClicked: {
                filterModel.enableLevel(1, !filterIncludesLevel)
            }
        }
        ToolButton {
            id: errorButton
            text: 'Errors'
            property bool filterIncludesLevel: true
            checkable: true
            checked: filterIncludesLevel
            onClicked: {
                filterModel.enableLevel(2, !filterIncludesLevel)
            }
        }
        ToolButton {
            id: staleButton
            text: 'Stale'
            property bool filterIncludesLevel: true
            checkable: true
            checked: filterIncludesLevel
            onClicked: {
                filterModel.enableLevel(3, !filterIncludesLevel)
            }
        }

    }

    Control {


        Layout.fillWidth: true
        Layout.fillHeight: true

        background: Rectangle {
            color: 'transparent'
            border.color: Qt.rgba(1, 1, 1, 0.2)
            border.width: 1
            radius: 6
        }

        padding: 12

        contentItem: TreeView {

            property var expandedPathSet: new Set()
            property bool restoringExpansion: false

            id: treeView

            clip: true

            model: DiagnosticFilterModel {
                id: filterModel
                filterText: searchField.text
                sourceModel: TreeModel {
                    id: treeModel
                }

                onAllowedLevelsChanged: {
                    infoButton.filterIncludesLevel = allowedLevels.includes(0)
                    warningButton.filterIncludesLevel = allowedLevels.includes(1)
                    errorButton.filterIncludesLevel = allowedLevels.includes(2)
                    staleButton.filterIncludesLevel = allowedLevels.includes(3)
                }


            }

            columnWidthProvider: function(column) {
                switch (column) {
                case 0: return -1 // name
                case 1: return 60  // level
                case 2: return treeView.width - treeView.columnWidth(0) - 60  // message
                default: return 0  // hide message, hardwareId, metrics
                }
            }

            selectionModel: ItemSelectionModel {}

            delegate: TreeViewDelegate {
                contentItem: Loader {
                    active: true
                    property list<Component> columnComponents: [
                        Component {
                            NameDelegate { }
                        },
                        Component {
                            LevelDelegate { }
                        },
                        Component {
                            NameDelegate { }
                        }
                    ]
                    sourceComponent: columnComponents[model.column]
                }
            }

            onExpanded: function(row, depth) {
                const index = treeView.index(row, 0)
                const path = treeView.model.data(index, TreeModel.PathRole)
                if (path)
                    expandedPathSet.add(path)

                if (!restoringExpansion)
                    Qt.callLater(function() { Logic.restoreExpandedPaths(Logic.expandedPaths()) })
            }

            onCollapsed: function(row, recursively) {
                const index = treeView.index(row, 0)
                const path = treeView.model.data(index, TreeModel.PathRole)
                if (path)
                    expandedPathSet.delete(path)
            }
        }
    }

}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "Diagnostics.js" as Logic

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

        ButtonGroup {
            buttons: [allButton, warningButton, errorButton]
        }

        Item {
            Layout.preferredWidth: 6
        }

        ToolButton {
            id: allButton
            text: 'Expand all'
            // checkable: true
            // checked: filterModel.minimumLevel <= 0
            onClicked: {
                filterModel.minimumLevel = 0
                Logic.expandAllVisible()
            }
        }
        ToolButton {
            id: warningButton
            text: 'Warnings'
            // checkable: true
            // checked: filterModel.minimumLevel === 1
            onClicked: Logic.setMinimumLevelFilter(1)
        }
        ToolButton {
            id: errorButton
            text: 'Errors'
            // checkable: true
            // checked: filterModel.minimumLevel === 2
            onClicked: Logic.setMinimumLevelFilter(2)
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

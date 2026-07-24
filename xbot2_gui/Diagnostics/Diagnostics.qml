import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Effects

import Main
import Common

import "Diagnostics.js" as Logic

Item {

    id: root

    property ClientEndpoint client

    RowLayout {

        anchors.fill: parent
        anchors.margins: 16
        spacing: 16

        ColumnLayout {
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: 400

            RowLayout {

                Layout.fillWidth: true

                SearchField {
                    id: searchField
                    Layout.fillWidth: true
                    suggestionModel: filterModel
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
                    text: 'All'
                    checkable: true
                    checked: true
                }
                ToolButton {
                    id: warningButton
                    text: 'Warnings'
                    checkable: true
                }
                ToolButton {
                    id: errorButton
                    text: 'Errors'
                    checkable: true
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
                }
            }

        }

        Item {

            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: 200

            Label {
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.centerIn: parent
                anchors.margins: 16
                visible: !treeView.selectionModel.currentIndex.valid
                text: 'No item selected'
                color: palette.disabled.text
                font.pixelSize: CommonProperties.font.h2
                wrapMode: Text.Wrap
            }

            DetailedView {

                anchors.fill: parent
                visible: treeView.selectionModel.currentIndex.valid

                message: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.MessageRole)
                hwId: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.HardwareIdRole)
                path: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.PathRole)
                name: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.NameRole)
                level: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.LevelRole)
                metrics: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.MetricsRole)

            }

        }

    }

    Connections {
        target: client
        function onObjectReceived(msg) {
            if(msg.type !== "diagnostics") {
                return
            }
            console.log('updating tree..')
            const cp = Logic.currentPath()
            const ep = Logic.expandedPaths()
            const sp = Logic.selectedPaths()
            treeModel.loadFromDiagnostics(msg)
            Logic.restoreExpandedPaths(ep)
            Logic.restoreSelectionState(sp, cp)
            console.log('..done')
        }
    }




}

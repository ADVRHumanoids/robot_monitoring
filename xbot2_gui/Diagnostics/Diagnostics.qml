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

    GridLayout {

        anchors.fill: parent
        anchors.margins: 16
        rowSpacing: 16
        columnSpacing: 16
        columns: 2

        DiagnosticsSummary {
            Layout.fillWidth: true
            Layout.columnSpan: 2
            Layout.preferredHeight: 250
            model: treeView.fullModel.activeIssues

            onFocusActiveIssue: function(path) {
                treeView.focusItem(path)
            }
        }

        DiagnosticsTreeView {
            id: treeView
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: 400
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
            treeView.loadData(msg)
        }
    }




}

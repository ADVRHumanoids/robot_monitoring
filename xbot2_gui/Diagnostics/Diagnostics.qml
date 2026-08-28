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
        columns: 1

        DiagnosticsSummary {
            // visible: !explorer.visible
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 250
            model: explorer.fullModel.activeIssues

            onFocusActiveIssue: function(path) {
                explorer.focusItem(path)
            }
        }

        // SectionHeader {
        //     Layout.fillWidth: true
        //     text: 'Diagnostics explorer'
        //     onClicked: explorer.visible = !explorer.visible
        // }

        DiagnosticsExplorer {
            id: explorer
            Layout.fillHeight: true
            Layout.fillWidth: true
        }

    }

    Connections {
        target: client
        function onObjectReceived(msg) {
            explorer.loadData(msg)
        }
    }




}

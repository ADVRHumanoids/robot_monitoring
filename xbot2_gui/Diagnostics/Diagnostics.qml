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

    Item {

        id: noDataItem

        anchors.fill: parent
        anchors.margins: 16

        Label {
            anchors.centerIn: parent
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            text: 'No diagnostics data received yet'
            color: palette.disabled.text
            font.pixelSize: CommonProperties.font.h2
        }
    }

    GridLayout {

        id: mainLayout

        anchors.fill: parent
        anchors.margins: 16
        rowSpacing: 16
        columnSpacing: 16
        columns: 1
        visible: !noDataItem.visible

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
            if(msg.type === 'diagnostics') {
                noDataItem.visible = false
                noDataTimer.restart()
            }
        }
    }

    Timer {
        id: noDataTimer
        interval: 5000
        repeat: false
        running: false
        onTriggered: {
            noDataItem.visible = true
        }
    }

}

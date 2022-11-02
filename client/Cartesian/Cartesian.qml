import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material
import NextUiModules

import "../Video"
import ".."
import "cartesian.js" as Logic

Page {

    id: root
    property ClientEndpoint client: undefined
    property var vref: [0, 0, 0, 0, 0, 0]
    property alias taskCombo: taskCombo

    RowLayout {

        anchors.fill: parent
        anchors.margins: 50
        spacing: 50

        ColumnLayout {
            id: leftColumn

            Pad {

                id: pad
                xLabel: 'YAW'
                yLabel: 'X'

                onJoystickMoved: {
                    vref[0] = joyY
                    vref[5] = -joyX
                    Logic.sendVref()
                }

            }
        }

        ColumnLayout {

            id: centerColumn

            Layout.fillWidth: true

            Row {
                id: centerColumnUpperRow
                Label {
                    text: 'Tasks'
                    anchors.verticalCenter: parent.verticalCenter
                }
                ComboBox {
                    anchors.verticalCenter: parent.verticalCenter
                    id: taskCombo
                    model: []
                }
                Button {
                    anchors.verticalCenter: parent.verticalCenter
                    text: 'Refresh'
                    onReleased: {
                        Logic.updateTaskNames()
                    }
                }
                Layout.fillWidth: true
                spacing: 10
            }

            VideoStreamFrontend {
                client: root.client

                Layout.rowSpan: 2
                Layout.columnSpan: 2
                width: parent.width
                height: parent.width / 1.33

                Layout.fillHeight: true
            }

            Item {
                width: centerColumnUpperRow.width
                height: centerColumnUpperRow.height
            }
        }

        ColumnLayout {
            id: rightColumn
            spacing: 30
            Pad {
                xLabel: 'Y'

                horizontalOnly: true

                onJoystickMoved: {
                    vref[1] = -joyX
                    Logic.sendVref()
                }

            }

            Pad {

                yLabel: 'Z'

                verticalOnly: true

                onJoystickMoved: {
                    vref[2] = joyY
                    Logic.sendVref()
                }

            }
        }
    }

    Component.onCompleted: {
        Logic.updateTaskNames()
    }
}

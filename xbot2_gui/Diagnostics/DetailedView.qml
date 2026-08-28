import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common

import "Diagnostics.js" as Logic

Control {

    property alias message: messageLabel.text
    property string hwId
    property alias name: nameLabel.text
    property alias path: pathLabel.text
    property alias status: statusLabel.text
    property int level: 3
    property list<var> metrics

    //
    id: root

    contentItem: ColumnLayout {

        spacing: 12

        // top banner
        RowLayout {

            Layout.fillWidth: true

            Column {
                spacing: 6
                Layout.fillWidth: true
                Label {
                    text: 'Selected diagnostic'
                    font.capitalization: Font.AllUppercase
                    font.bold: true
                    color: palette.accent
                    width: parent.width
                }
                Label {
                    id: nameLabel
                    font.pixelSize: CommonProperties.font.h2
                    font.bold: true
                    width: parent.width
                }
                Label {
                    id: pathLabel
                    color:  palette.disabled.text
                    wrapMode: Text.Wrap
                    width: parent.width
                }
            }

            // status badge
            Label {
                id: statusLabel
                background: Rectangle {
                    radius: height / 2.
                    color: Qt.darker(statusLabel.color)
                }
                color: enabled ? Logic.levelToColor(root.level) : 'grey'
                text: Logic.levelToText(root.level)
                font.bold: true
                padding: 6
                leftPadding: 12
                rightPadding: 12
            }
        }

        // hw id badge
        Label {
            id: hwIdLabel
            text: `Hardware ID: <b>${root.hwId}</b>`
            padding: 6
            background: Rectangle {
                color: Qt.rgba(0, 0, 0, 0.1)
                border.color: Qt.rgba(1, 1, 1, 0.2)
                border.width: 1
                radius: height / 4
            }
        }

        // message
        Control {
            Layout.fillWidth: true
            padding: 12
            contentItem: Column {
                spacing: 8
                Label {
                    text: 'MESSAGE'
                    font.bold: true
                }
                Label {
                    id: messageLabel
                    font.pixelSize: CommonProperties.font.h4
                    wrapMode: Text.Wrap
                    width: parent.width
                }
            }
            background: Rectangle {
                color: Qt.rgba(0, 0, 0, 0.1)
                border.color: Qt.rgba(1, 1, 1, 0.2)
                border.width: 1
                radius: height / 6
            }
        }

        // metrics
        Label {
            text: 'Metrics'
            font.bold: true
            font.pixelSize: CommonProperties.font.h3
            visible: root.metrics.length > 0
        }

        //
        ScrollView {
            id: metricsScroll
            Layout.fillWidth: true
            Layout.fillHeight: true
            contentWidth: availableWidth
            GridLayout {
                id: grid
                width: metricsScroll.contentWidth
                columns: Math.ceil(width / 300)
                rows: Math.ceil(root.metrics.length / columns)
                rowSpacing: 12
                columnSpacing: 12

                uniformCellHeights: true
                uniformCellWidths: true

                Repeater {
                    model: root.metrics
                    delegate: FramedValue {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: modelData.key
                        value1: Logic.formatValue(modelData.value)
                        radius: 8
                    }
                }
            }
        }

        // spaces
        // Item {
        //     implicitHeight: 1
        //     Layout.fillHeight: true
        // }

    }
}

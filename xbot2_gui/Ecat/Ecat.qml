import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Main
import Common

import "Ecat.js" as Logic

Control {

    property ClientEndpoint client

    signal pageSelected()

    //
    id: root

    property list<string> sdoCmds: ['MOTOR_ON', 'MOTOR_OFF']
    property list<string> allSdo: ['ciao_miao', 'gattoBello', 'zio_buOno']
    property list<string> allId: ['ALL'].concat([...new Array(40).keys()].map(i => i.toString()))
    property list<string> selectedSdo
    property list<string> selectedId

    padding: 4

    contentItem: ScrollView {
        id: scroll
        contentWidth: availableWidth
        ColumnLayout {

            width: scroll.contentWidth

            spacing: CommonProperties.geom.margins

            id: content

            // select ids
            GridLayout {

                id: mainGrid

                columns: layout.expanded ? 4 : 2
                columnSpacing: 16
                Layout.fillWidth: true

                //
                Label {
                    text: 'Select ID'
                    font.bold: true
                }
                ComboBox {
                    id: idCombo
                    model: allId
                    Layout.fillWidth: true
                    Layout.columnSpan: 2
                }
                Button {
                    text: 'Add'
                    Layout.fillWidth: true
                    Layout.columnSpan: layout.expanded ? 1 : 2
                    onClicked: {

                        if(idCombo.currentText === 'ALL') {
                            selectedId = allId.slice(1)
                        } else {
                            selectedId.push(idCombo.currentText)
                            selectedId = [...new Set(selectedId)]
                        }

                        content.updateGrid()
                    }
                }

                //
                Label {
                    text: 'Select SDO'
                    font.bold: true
                }
                ComboBox {
                    id: sdoCombo
                    model: allSdo
                    Layout.fillWidth: true
                    Layout.columnSpan: layout.expanded ? 1 : 2
                }
                TextField {
                    placeholderText: 'Search SDO'
                    Layout.fillWidth: true
                    Layout.columnSpan: layout.expanded ? 1 : 2
                    onTextEdited: {
                        sdoCombo.model = allSdo.filter(item => item.toLowerCase().indexOf(text.toLowerCase()) !== -1)
                    }
                }

                Button {
                    Layout.fillWidth: true
                    Layout.columnSpan: layout.expanded ? 1 : 2
                    text: 'Add'
                    onClicked: {
                        selectedSdo.push(sdoCombo.currentText)
                        selectedSdo = [...new Set(selectedSdo)]
                        content.updateGrid()
                    }
                }

                //
                Label {
                    text: 'SDO Command / ID'
                    font.bold: true
                    enabled: writeCheck.checked
                    Layout.columnSpan: layout.expanded ? 1 : 2
                }

                ComboBox {
                    id: cmdCombo
                    model: sdoCmds
                    Layout.fillWidth: true
                    Layout.columnSpan: layout.expanded ? 1 : 2
                    enabled: writeCheck.checked
                }

                ComboBox {
                    id: cmdId
                    model: allId
                    Layout.fillWidth: true
                    Layout.columnSpan: layout.expanded ? 1 : 2
                    enabled: writeCheck.checked
                }

                Button {
                    id: cmdSendBtn
                    text: 'Send'
                    Layout.fillWidth: true
                    enabled: writeCheck.checked
                    Layout.columnSpan: layout.expanded ? 1 : 2
                }

                ToolSeparator {
                    Layout.fillWidth: true
                    Layout.columnSpan: mainGrid.columns
                    orientation: Qt.Horizontal
                }

                // bottom btns
                GridLayout {
                    Layout.fillWidth: true
                    columnSpacing: 8
                    rows: layout.expanded ? 1 : -1
                    columns: layout.expanded ? -1 : 1
                    Layout.columnSpan: mainGrid.columns
                    Button {
                        Layout.fillWidth: true
                        text: 'Refresh'
                    }
                    Button {
                        Layout.fillWidth: true
                        text: 'Clear IDs'
                    }
                    Button {
                        Layout.fillWidth: true
                        text: 'Clear SDOs'
                    }
                    Button {
                        Layout.fillWidth: true
                        text: 'Read all'
                    }
                    Switch {
                        id: writeCheck
                        text: 'Write mode'
                        enabled: CommonProperties.config.adminPwdOk
                    }
                }
            }

            // grid
            GridLayout {
                Layout.fillHeight: true
                Layout.fillWidth: true
                id: sdoGrid
                columns: Math.max(1, selectedSdo.length) + 2
                rowSpacing: -4
                columnSpacing: 8
            }

            function updateGrid() {
                sdoGrid.children = []

                // col header
                empty.createObject(sdoGrid, {'text': 'ID'})

                for(const sdo of selectedSdo) {
                    sdoHdr.createObject(sdoGrid, {'text': sdo})
                }
                if(selectedSdo.length === 0) {
                    sdoHdr.createObject(sdoGrid, {'text': '--', 'enabled': false})
                }

                empty.createObject(sdoGrid)

                // some space
                for(let i = 0; i < sdoGrid.columns; i++) {
                    empty.createObject(sdoGrid)
                }

                // rows
                for(const id of selectedId) {
                    idHdr.createObject(sdoGrid, {'text': id})

                    for(const sdo of selectedSdo) {
                        sdoDelegate.createObject(sdoGrid)
                    }

                    if(selectedSdo.length === 0) {
                        sdoDelegate.createObject(sdoGrid, {'enabled': false})
                    }

                    readBtn.createObject(sdoGrid)

                }
            }

            property Component empty: Label {

            }

            property Component idHdr: Label {
                Layout.preferredWidth: 30
                padding: 4
                horizontalAlignment: TextEdit.AlignRight
                MouseArea {
                    id: mouse
                    anchors.fill: parent
                    onDoubleClicked: {
                        selectedId = selectedId.filter(item => item !== text)
                        content.updateGrid()
                    }
                    hoverEnabled: true
                }
                background: Rectangle {
                    radius: 4
                    color: Qt.rgba(1, 1, 1, 0.1*(1 + mouse.containsMouse))
                }
            }

            property Component sdoHdr: Label {
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        selectedSdo = selectedSdo.filter(item => item !== text)
                        content.updateGrid()
                    }
                }
            }

            property Component sdoDelegate: TextField {
                Layout.fillWidth: true
                background.implicitHeight: 30
                text: '--'
                readOnly: !writeCheck.checked
            }

            property Component readBtn: Button {
                background.implicitHeight: 30
                text: writeCheck.checked ? 'Write' : 'Read'
            }

        }

    }

    onPageSelected: {
        Logic.construct()
    }

}

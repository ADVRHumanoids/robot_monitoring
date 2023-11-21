import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "../Common"
import Common

Card1 {

    property bool jointActive: false
    property real filterCutoff: 0.0
    property bool filterActive: true

    signal setSafetyState(bool ok)
    signal setFilterCutoff(string profile)
    signal setFilterActive(bool ok)

    id: root

    backgroundColor: jointActive ? CommonProperties.colors.ok :
                                   CommonProperties.colors.err

    configurable: false

    name: 'Safety'

    toolButtons: [
        Button {
            text: jointActive ? 'Stop' : 'Restore'
            onClicked: root.setSafetyState(!jointActive)
        }
    ]

    frontItem: GridLayout {

        id: grid
        anchors.fill: parent
        columns: 2
        columnSpacing: root.margins

        Label {
            text: 'Cutoff [Hz]'
        }

        TextField {
            readOnly: true
            text: filterCutoff
        }

        GroupBox {
            title: 'Filter Settings'
            Layout.fillWidth: true
            Layout.columnSpan: 2
            enabled: jointActive
            ColumnLayout {
                spacing: -6
                CheckBox {
                    id: filterCheck
                    text: 'Enabled'
                    checked: filterActive
                    onReleased: {
                        root.setFilterActive(checked)
                    }
                }

                RadioButton {
                    id: safe
                    text: 'Safe'
                    enabled: filterCheck.checked
                    checked: filterCutoff <= 2.0
                    onReleased: {
                        if(checked) root.setFilterCutoff('safe')
                    }
                }
                RadioButton {
                    id: medium
                    text: 'Medium'
                    enabled: filterCheck.checked
                    checked: filterCutoff <= 10.0 && !safe.checked
                    onReleased: {
                        if(checked) root.setFilterCutoff('medium')
                    }
                }
                RadioButton {
                    text: 'Fast'
                    enabled: filterCheck.checked
                    checked: filterCutoff > 10.0
                    onReleased: {
                        if(checked) root.setFilterCutoff('fast')
                    }
                }
            }
        }
    }
}

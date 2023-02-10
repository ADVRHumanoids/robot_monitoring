import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

Card {

    property bool jointActive: false
    property real filterCutoff: 0.0
    property bool filterActive: true

    signal triggerSafety(bool ok)

    id: root

    backgroundColor: jointActive ? CommonProperties.colors.ok :
                                   CommonProperties.colors.err

    configurable: false

    name: 'Safety'

    toolButtons: [
        Switch {
            checked: jointActive
            onClicked: root.triggerSafety(checked)
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
                }

                RadioButton {
                    id: safe
                    text: 'Safe'
                    enabled: filterCheck.checked
                    checked: filterCutoff <= 2.0
                }
                RadioButton {
                    id: medium
                    text: 'Medium'
                    enabled: filterCheck.checked
                    checked: filterCutoff <= 10.0 && !safe.checked
                }
                RadioButton {
                    text: 'Fast'
                    enabled: filterCheck.checked
                    checked: filterCutoff > 10.0
                }
            }
        }
    }
}

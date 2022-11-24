import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

Rectangle {

    property bool jointActive: false
    property real filterCutoff: 0.0
    property bool filterActive: true

    color: jointActive ? CommonProperties.colors.cardBackground :
                         CommonProperties.colors.err
    radius: 4
    implicitHeight: grid.implicitHeight + 32
    implicitWidth: grid.implicitWidth + 32

    GridLayout {

        id: grid
        anchors.fill: parent
        anchors.margins: 16
        rows: 3
        columns: 2

        Label {
            id: label
            text: 'Joint Device'
            font.pixelSize: CommonProperties.font.h1
        }

        Switch {
//            text: jointActive ? 'Disable' : 'Enable'
            checked: jointActive
        }

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

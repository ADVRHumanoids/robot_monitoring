import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Common
import QtCore

import "ConfigurePanel.js" as Logic
import "../TestThings"

Item {
    // public

    property var description: Object()

    property var options: Object()

    property var info: Object()

    function applySettings() {
        Logic.apply()
    }

    signal procEditRequested()

    // private

    id: root

    property var _controls: Object()
    property alias _grid: mainLayout

    implicitHeight: mainColumn.implicitHeight
    implicitWidth: mainColumn.implicitWidth

    // layout to be filled by Logic.construct()
    ColumnLayout {

        id: mainColumn
        anchors.fill: parent
        spacing: 16

        GridLayout {

            Layout.fillWidth: true

            columns: 1

            // command
            FramedValue {
                title: 'Command'
                value1: root.info.cmd || ''
                Layout.fillWidth: true
            }

            // machine
            FramedValue {
                title: 'Machine'
                value1: root.info.machine || ''
                Layout.fillWidth: true
            }

            // docker
            FramedValue {
                visible: root.info.docker !== undefined && root.info.docker !== ''
                title: 'Container'
                value1: root.info.docker || ''
                Layout.fillWidth: true
            }

            // edit btn
            Button {
                text: 'Edit process'
                Layout.fillWidth: true
                onClicked: root.procEditRequested()
                visible: root.info.is_custom
            }
        }

        GridLayout {
            id: mainLayout
            columns: 2
            columnSpacing: 16
            Layout.fillHeight: true
            Layout.fillWidth: true
        }

    }

    property var label: Component {

        Label {

        }

    }

    property var combo: Component {

        ComboBox {

        }

    }

    property var check: Component {

        CheckBox {

        }

    }

    property var text: Component {

        TextArea {

        }

    }

    Settings {
        id: settings
    }

    // initialization
    onDescriptionChanged: {
        Logic.construct()
    }

}

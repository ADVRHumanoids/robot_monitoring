import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Control {

    property list<string> availableMachines
    property list<string> availableContainers

    function setEditMode(info) {
        _prevName = info.name
        procNameField.text = info.name
        machineCombo.editText = info.machine
        dockerCombo.editText = info.container
        cmdField.text = info.cmd
        visibleChk.checked = info.visible
        _editMode = true
    }

    signal addCustomProcess(string name, string machine, string container, string cmd, bool visible, bool edit, string prevName)
    signal closeRequest()

    leftPadding: width > 800 ? (width - 800) / 2 : (layout.compact ? 0 : 16)
    rightPadding: leftPadding
    bottomPadding: 16

    //
    id: root
    property bool _editMode: false
    property string _prevName

    contentItem: GridLayout {

        columns: 2

        rowSpacing: 8
        columnSpacing: 8

        // title
        Label {
            text: _editMode ? 'Edit custom process' : 'Add custom process'
            font.pixelSize: CommonProperties.font.h2
            Layout.columnSpan: 2
        }

        // subtitle
        Label {
            text: 'This process will be available to all connected clients.'
            font.pixelSize: CommonProperties.font.h4
            Layout.columnSpan: 2
            Layout.fillWidth: true
            wrapMode: Text.Wrap

            bottomPadding: 8
        }

        // 0

        Label {
            text: 'Name'
        }

        TextField {
            id: procNameField
            Layout.fillWidth: true
            placeholderText: 'Process name'
        }

        // 1

        Label {
            text: 'Machine'
        }

        ComboBox {
            Layout.fillWidth: true
            id: machineCombo
            model: root.availableMachines
            editable: true
        }

        // 2

        Label {
            text: 'Container'
        }

        ComboBox {
            Layout.fillWidth: true
            id: dockerCombo
            model: root.availableContainers
            editable: true
        }

        // 4

        FramedControl {
            Layout.fillWidth: true
            Layout.columnSpan: 2
            text: 'Visible'
            subtext: 'The process will be visible by default.'
            CheckBox {
                id: visibleChk
            }
        }

        // 3

        TextField {
            Layout.fillWidth: true
            id: cmdField
            Layout.columnSpan: 2
            placeholderText: 'Shell command'
        }

        // 5

        RowLayout {

            Layout.columnSpan: 2
            spacing: 8
            Layout.fillWidth: true

            Button {
                Layout.fillWidth: true
                text: _editMode ? 'Submit' : 'Add'
                enabled: procNameField.text.length > 0 && cmdField.text.length > 0
                onClicked: {
                    root.addCustomProcess(procNameField.text,
                                          machineCombo.editText,
                                          dockerCombo.editText,
                                          cmdField.text,
                                          visibleChk.checked,
                                          _editMode,
                                          _prevName)
                }
            }

            Button {
                text: 'Close'
                Layout.fillWidth: true
                onClicked: root.closeRequest()
            }

        }

    }

}



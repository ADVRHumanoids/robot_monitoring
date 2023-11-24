import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Card1 {

    property list<string> availableMachines

    property Item pageItem

    signal submitCommand(string machine, string command, int timeout)

    onSubmitCommand: {
        stdoutText.clear()
        stderrText.clear()
    }

    function setResult(retcode, stdout, stderr) {

        stdoutText.append(stdout)
        stderrText.append(stderr)
    }

    id: root

    name: 'Custom command'
    nameFont.pixelSize: CommonProperties.font.h3

    collapsable: false

    collapsed: true

    configurable: false

    toolButtons: [

        Button {
            text: 'Submit'
            onClicked: popup.open()
        }

    ]

    Popup {

        id: popup
        anchors.centerIn: Overlay.overlay
        width: Math.min(Overlay.overlay.width - 48, 600)
        height: Math.min(Overlay.overlay.height - 48, implicitHeight)
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        padding: 24

        GridLayout {

            anchors.fill: parent

            columns: 2

            rowSpacing: 8
            columnSpacing: 8

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

            TextField {
                Layout.fillWidth: true
                id: cmdField
                Layout.columnSpan: 2
                placeholderText: 'Shell command'
                onAccepted: root.submitCommand(machineCombo.currentText,
                                               cmdField.text,
                                               timeoutSpin.value)
                focus: true
            }

            // 3

            Label {
                text: 'Timeout'
            }

            SpinBox {
                Layout.fillWidth: true
                id: timeoutSpin
                from: 1
                to: 100
                value: 5
            }

            // 4

            Button {
                Layout.columnSpan: 2
                Layout.fillWidth: true
                text: 'Submit'
                onClicked: root.submitCommand(machineCombo.currentText,
                                              cmdField.text,
                                              timeoutSpin.value)
            }

            //

            ScrollView {

                id: scroll
                Layout.columnSpan: 2
                Layout.fillWidth: true
                Layout.fillHeight: true
                TextArea {
                    id: stdoutText
                    placeholderText: 'Stdout'
                    readOnly: true
                    width: scroll.availableWidth
                }

            }

            //

            TextArea {
                id: stderrText
                Layout.columnSpan: 2
                Layout.fillWidth: true
                placeholderText: 'Stderr'
                readOnly: true
                color: CommonProperties.colors.err
            }

        }

        Behavior on height {
            NumberAnimation {
                duration: 333
                easing.type: Easing.OutQuad
            }
        }

    }

}

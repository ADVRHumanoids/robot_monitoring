import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Common

import "Drilling.js" as Logic

Control {

    property alias motionTarget: moveCombo.currentText
    property alias maxSpeed: maxSpeedLinearSpinBox.value
    property alias joyXEnabled: chkX.checked
    property alias joyYEnabled: chkY.checked
    property alias joyZEnabled: chkZ.checked

    //
    id: root
    padding: 4
    topPadding: 8
    bottomPadding: 8

    contentItem: ColumnLayout {

        id: col

        spacing: 3

        Label {
            text: ' joy target'
            font.pointSize: 10
            enabled: moveEnableSwitch.checked
        }

        RowLayout {

            Layout.fillWidth: true

            ComboBox {
                id: moveCombo
                model: ['Arm', 'Base']
                Layout.fillWidth: true
                enabled: moveEnableSwitch.checked

                onCurrentTextChanged: {
                    if(currentText === 'Arm') {
                        Logic.enableArmControl(true)
                    }
                }
            }

            Switch {
                visible: false
                checked: true
                id: moveEnableSwitch
            }

        }

        Item {
            width: parent.width
            height: 3
        }

        Label {
            enabled: moveEnableSwitch.checked
            text: ' max speed'
            font.pointSize: 10
        }

        DoubleSpinBox {
            Layout.fillWidth: true
            width: parent.width
            enabled: moveEnableSwitch.checked
            id: maxSpeedLinearSpinBox
            from: 0.0
            to: 2.0
            value: 0.2
        }

        Item {
            width: parent.width
            height: 3
        }

        Label {
            enabled: moveEnableSwitch.checked
            text: ' enabled directions'
            font.pointSize: 10
        }

        Row {
            enabled: moveEnableSwitch.checked
            topPadding: -6
            bottomPadding: -6
            CheckBox {
                id: chkX
                text: 'X'
                checked: true
                onCheckedChanged: chkZ.checked = chkZ.checked && !checked
            }
            CheckBox {
                id: chkY
                text: 'Y'
                checked: true
            }
            CheckBox {
                id: chkZ
                text: 'Z'
                checked: false
                onCheckedChanged: chkX.checked = chkX.checked && !checked
            }
        }

        Item {
            Layout.fillHeight: true
        }

        Button {
            Layout.preferredWidth: impBtn.width
            text: 'Gcomp'
            Layout.alignment: Qt.AlignHCenter
            onClicked: Logic.enableGcomp(true)
        }

        Button {
            id: impBtn
            text: 'Impedance'
            Layout.alignment: Qt.AlignHCenter
            onClicked: Logic.enableGcomp(false)
        }


    }

}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Main
import Common
import Monitoring.BarPlot

import "JointCommand.js" as Logic

Card1 {

    // property ClientEndpoint client

    property alias gripperNames: gripperCombo.model

    name: 'Gripper Command'

    frontItem: GridLayout {

        anchors.fill: parent

        columns: 2

        RowLayout {
            Layout.fillWidth: true
            Layout.columnSpan: 2
            ComboBox {
                id: gripperCombo
                model: ['Gripper1', 'Gripper2']
                Layout.fillWidth: true
            }

            Button {
                text: 'Refresh'
                onClicked: Logic.updateGripperNames()
            }
        }


        Item {
            Layout.fillWidth: true
            Layout.columnSpan: 2
            Layout.preferredHeight: 8
        }


        // Label {
        //     text: 'Position'
        // }

        // TwoSideBar {
        //     id: posBar
        //     Layout.fillWidth: true
        // }


        // Label {
        //     text: 'Effort'
        // }

        // TwoSideBar {
        //     id: torBar
        //     Layout.fillWidth: true
        // }


        // Item {
        //     Layout.fillWidth: true
        //     Layout.columnSpan: 2
        //     Layout.preferredHeight: 8
        // }

        RowLayout {

            Layout.fillWidth: true
            Layout.columnSpan: 2

            Label {
                text: 'Effort reference'
            }

            Slider {
                Layout.fillWidth: true
                id: slider

                onMoved: {

                }

                from: -10
                to: 10

            }

            TextField {
                id: sliderLabel
                text: slider.value.toFixed(2)
                onTextEdited: {
                    slider.value = parseFloat(text)
                }
            }

        }

        RowLayout {

            Layout.fillWidth: true
            Layout.columnSpan: 2

            Button {
                Layout.fillWidth: true
                Layout.preferredWidth: 1
                text: 'Open'
                onClicked: Logic.sendGripperCommand(gripperCombo.currentText, 'open')
            }

            Button {
                Layout.fillWidth: true
                Layout.preferredWidth: 1
                text: 'Apply force'
                onClicked: Logic.sendGripperCommand(gripperCombo.currentText, 'close', slider.value)
            }

        }


    }

    Component.onCompleted: Logic.updateGripperNames()

    // Connections {
    //     target: client
    //     function onObjectReceived(msg) {
    //         if(msg.type === 'gripper_state') {
    //             if(msg.name === gripperCombo.currentText) {

    //             }
    //         }
    //     }
    // }

}

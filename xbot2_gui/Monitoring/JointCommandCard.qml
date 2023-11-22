import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main
import Viewer3D
import "/qt/qml/Main/sharedData.js" as SharedData

import "JointCommand.js" as Logic

Card {

    property ClientEndpoint client
    property RobotModelNode robotCmd
    signal resetCmd()
    function selectJoint(jname) {
        nameCombo.currentIndex = nameCombo.find(jname)
    }


    // private
    id: root
    name: 'Joint Command'
    configurable: true


    frontItem: GridLayout {

        id: ctrlGrid

        anchors.fill: parent

        columns: 2

        columnSpacing: 16
        rowSpacing: 16


        ComboBox {
            Layout.fillWidth: true
            id: nameCombo
            Layout.columnSpan: 2
            model: robotCmd.jointNames
            onCurrentIndexChanged: {
                root.resetCmd()
                slider.value = robotCmd.q[currentIndex]
            }
        }

        Slider {
            Layout.fillWidth: true
            id: slider
            Layout.columnSpan: 2
            onMoved: {
                robotCmd.q[nameCombo.currentIndex] = value
                robotCmd.qChanged()
            }
            from: SharedData.qmin[SharedData.jointNames.indexOf(nameCombo.currentText)]
            to: SharedData.qmax[SharedData.jointNames.indexOf(nameCombo.currentText)]

            Rectangle {
                anchors.centerIn: sliderLabel
                width: sliderLabel.width + 8
                height: sliderLabel.height + 8
                color: Qt.rgba(0, 0, 0, 0.3)
                radius: 4
            }


            Label {
                id: sliderLabel
                anchors.centerIn: parent
                text: parent.value.toFixed(2)
                z: 1
            }
        }

        Button {
            id: trjCmdBtn
            property bool running: false
            Layout.columnSpan: 1
            Layout.fillWidth: true
            text: running ? 'Stop' : 'Send'
            onReleased: {
                if(running) {
                    Logic.stopCommand()
                }
                else {
                    running = true
                    Logic.sendCommand(nameCombo.currentText,
                                      slider.value,
                                      trjTimeSpin.value)
                }
            }
        }

        Button {
            Layout.columnSpan: 1
            Layout.fillWidth: true
            text: 'Reset'
            onReleased: {
                root.resetCmd()
                nameCombo.currentIndexChanged()
            }
        }

        Item {
            Layout.fillHeight: true
        }
    }

    backItem: Control {

        contentItem: Column {

            spacing: 6

            Label {
                text: 'Trajectory time'
            }

            DoubleSpinBox {
                id: trjTimeSpin
                from: 1.0
                to: 10.0
                stepSize: 1.0
                value: 5.0
            }
        }

    }
}

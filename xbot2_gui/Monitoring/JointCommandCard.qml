import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main
import ViewerQuick3D
import "/qt/qml/Main/sharedData.js" as SharedData

import "JointCommand.js" as Logic

Card1 {

    property ClientEndpoint client
    property RobotModelNode robotCmd
    signal resetCmd()
    signal cmdChanged()
    signal jointRemoved()
    property list<string> ctrlJoints
    property alias activeCtrl: ctrlCombo.currentText
    property alias enableMultipleSelection: multiJointChk.checked

    function selectJoint(jName) {
        if(multiJointChk.checked) {
            ctrlJoints.push(jName)
            ctrlJoints = [...new Set(ctrlJoints)]
        }
        else {
            ctrlJoints = [jName]
        }
    }

    function removeJoint(jName) {
        ctrlJoints = ctrlJoints.filter(n => n !== jName)
        jointRemoved()
    }


    // private
    id: root
    name: 'Joint Command'
    property bool continuousPublishMode: activeCtrl === 'Velocity' || activeCtrl === 'Effort'
    configurable: true

    onCtrlJointsChanged: {
        Qt.callLater( () => {
            slider.value = Logic.currentValue(root.ctrlJoints, root.activeCtrl)
        })
        root.resetCmd()
    }

    toolButtons: [
        ComboBox {
            id: ctrlCombo
            model: Logic.cmdFieldsLong
            onCurrentIndexChanged: root.ctrlJointsChanged()
        }
    ]

    frontItem: GridLayout {

        id: ctrlGrid

        anchors.fill: parent

        columns: 2

        columnSpacing: 16
        rowSpacing: 8

        Label {
            text: root.ctrlJoints.length > 0 ? 'Selected joints:' : 'No joint selected'
            Layout.columnSpan: 2
        }

        Flow {
            spacing: 4
            Layout.columnSpan: 2
            Layout.fillWidth: true

            Repeater {
                id: jointRepeater
                model: root.ctrlJoints
                Label {
                    padding: 4
                    text: modelData
                    background: Rectangle {
                        color: Qt.rgba(1, 1, 1, 0.1)
                        radius: 2
                    }
                    MouseArea {
                        anchors.fill: parent
                        onClicked: root.removeJoint(modelData)
                    }
                    Component.onCompleted: {
                        selectAllBtn.height = height
                        clearBtn.height = height
                    }
                }
            }

            ToolButton {
                id: selectAllBtn
                text: 'ALL'
                visible: jointRepeater.count < SharedData.jointNames.length
                onClicked: root.ctrlJoints = SharedData.jointNames
            }

            ToolButton {
                id: clearBtn
                text: 'X'
                visible: jointRepeater.count > 1
                onClicked: {root.ctrlJoints = []; root.jointRemoved()}
            }
        }

        RowLayout {

            visible: jointDevice.jointActive

            Layout.fillWidth: true
            Layout.columnSpan: 2

            Slider {
                enabled: root.ctrlJoints.length > 0
                Layout.fillWidth: true
                id: slider

                onMoved: {
                    if(root.activeCtrl === 'Position') {
                        robotCmd.q = Logic.updateQ(robotCmd.q, root.ctrlJoints)
                        robotCmd.qChanged()
                        root.cmdChanged()
                    }
                }
                from: Logic.sliderRange(root.ctrlJoints, root.activeCtrl)[0]
                to: Logic.sliderRange(root.ctrlJoints, root.activeCtrl)[1]

            }

            TextField {
                enabled: root.ctrlJoints.length > 0
                id: sliderLabel
                text: slider.value.toFixed(2)
                onEditingFinished: {
                    slider.value = parseFloat(text)
                }
            }

        }

        Button {
            id: trjCmdBtn
            visible: jointDevice.jointActive
            property bool running: false
            enabled: root.ctrlJoints.length > 0
            Layout.columnSpan: 1
            Layout.fillWidth: true
            text: running ? 'Stop' : 'Send'
            onPressed: {
                if(root.continuousPublishMode) {
                    velTorTimer.start()
                }
            }

            onReleased: {

                if(root.continuousPublishMode) {
                    velTorTimer.stop()
                    return
                }

                if(running) {
                    Logic.stopCommand()
                }
                else {
                    running = true
                    Logic.sendCommand(root.ctrlJoints,
                                      root.activeCtrl,
                                      slider.value,
                                      trjTimeSpin.value)
                }
            }
        }

        Button {
            visible: jointDevice.jointActive
            enabled: root.ctrlJoints.length > 0
            Layout.columnSpan: 1
            Layout.fillWidth: true
            text: 'Reset'
            onReleased: {
                root.ctrlJointsChanged()
            }
        }

        DelayButton {
            visible: !jointDevice.jointActive
            enabled: root.ctrlJoints.length > 0
            text: 'Stop Motor'
            Layout.fillWidth: true
            onActivated: {
                progress = 0;
                Logic.stopMotor();
            }
            delay: 333
        }

        DelayButton {
            visible: !jointDevice.jointActive
            enabled: root.ctrlJoints.length > 0
            text: 'Start Motor'
            Layout.fillWidth: true
            onActivated: {
                progress = 0;
                Logic.startMotor()
            }
            delay: 333
        }

        DelayButton {
            visible: !jointDevice.jointActive
            enabled: root.ctrlJoints.length > 0
            text: 'Engage brake'
            Layout.fillWidth: true
            onActivated: {
                progress = 0;
                Logic.engageBrake()
            }
            delay: 333
        }

        DelayButton {
            visible: !jointDevice.jointActive
            enabled: root.ctrlJoints.length > 0
            text: 'Release brake'
            Layout.fillWidth: true
            onActivated: {
                progress = 0;
                Logic.releaseBrake()
            }
            delay: 333
        }


        Item {
            Layout.fillHeight: true
        }

        Timer {
            id: velTorTimer
            interval: 20
            repeat: true
            onTriggered: {
                Logic.sendContinuousCommand(root.ctrlJoints,
                                            root.activeCtrl,
                                            slider.value)
            }
        }
    }

    backItem: Control {

        contentItem: GridLayout {

            columns: 2

            columnSpacing: 6
            rowSpacing: 6

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

            CheckBox {
                Layout.columnSpan: 2
                id: multiJointChk
                checked: false
                text: 'Enable multiple joints'
            }
        }

    }
}

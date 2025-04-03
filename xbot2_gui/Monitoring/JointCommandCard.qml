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
    property list<string> ctrlJoints
    property alias activeCtrl: ctrlCombo.currentText

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
    }


    // private
    id: root
    name: 'Joint Command'
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
        rowSpacing: 16


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
                    Component.onCompleted: clearBtn.height = height
                }
            }
            ToolButton {
                id: clearBtn
                text: 'X'
                visible: jointRepeater.count > 1
                onClicked: root.ctrlJoints = []
            }
        }

        RowLayout {

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
                onAccepted: {
                    slider.value = parseFloat(text)
                }
            }

        }

        Button {
            id: trjCmdBtn
            property bool running: false
            enabled: root.ctrlJoints.length > 0
            Layout.columnSpan: 1
            Layout.fillWidth: true
            text: running ? 'Stop' : 'Send'
            onReleased: {
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
            enabled: root.ctrlJoints.length > 0
            Layout.columnSpan: 1
            Layout.fillWidth: true
            text: 'Reset'
            onReleased: {
                root.ctrlJointsChanged()
            }
        }

        Item {
            Layout.fillHeight: true
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

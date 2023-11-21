import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Common
import "Joy.js" as Logic
import "../Video/VideoStream.js" as VideoStream


Card1 {

    name: 'Joy Setup'
    property alias currentTask: taskCombo.currentText
    property alias linXOnly: linXCheck.checked
    property real maxSpeed: maxSpeedLinearSpinBox.value
    property bool ikRunning: false
    property alias videoStream: videoStreamCombo.currentText

    // private
    id: root
    configurable: false

    toolButtons: [
        Label {
            text: ikRunning ? `Current task:  <i>${currentTask}</i>` :
                               'IK is not running'
        },
        Button {
            text: 'Refresh'
            onReleased: {
                Logic.updateTaskNames(taskCombo)
                Logic.taskIsEnabled(taskCombo.currentText,
                                    (is_active) => {
                                        grid.taskActive = is_active
                                    })
            }
        }
    ]

    frontItem: GridLayout {

        id: grid

        enabled: ikRunning

        anchors.fill: parent

        columns: 3

        Label {
            text: 'Tasks'
        }

        ComboBox {
            id: taskCombo
            model: []
            Layout.fillWidth: true
            onCurrentTextChanged: {
                // get state from server to update enable/disable btn
                Logic.taskIsEnabled(currentText,
                                    function (is_active) {
                                        grid.taskActive = is_active
                                    })
            }
            onModelChanged: {
                currentIndex = indexOfValue(settings.currentTask)
            }
        }

        property bool taskActive: false

        Button {
            id: enableDisableBtn
            text: grid.taskActive ? 'Disable' : 'Enable'
            onReleased: {
                let callback = function () {
                    Logic.taskIsEnabled(taskCombo.currentText,
                                        (is_active) => {
                                            grid.taskActive = is_active
                                        })
                }
                if(grid.taskActive) {
                    Logic.disableTask(taskCombo.currentText, callback)
                }
                else {
                    Logic.enableTask(taskCombo.currentText, callback)
                }
            }
        }



        Label {
            text: 'Max speed (linear)'
        }

        DoubleSpinBox {
            id: maxSpeedLinearSpinBox
            from: 0.0
            to: 2.0
        }

        CheckBox {
            id: linXCheck
            text: 'Linear X Only'
            checked: false
        }

        Label {
            text: 'Video stream'
        }

        ComboBox {
            id: videoStreamCombo
            Layout.fillWidth: true
        }

        Button {

            text: 'Refresh'
            onClicked: {
                VideoStream.refreshNames(undefined,
                                         (topics) => {
                                             videoStreamCombo.model = topics
                                         })
            }
        }
    }

    Settings {
        id: settings
        category: 'JoyCartesianCard'
        property string currentTask
    }

//    Timer {
//        repeat: true
//        interval: 5000
//        onTriggered: Logic.updateTaskNames(taskCombo)
//        Component.onCompleted: start()
//    }

    Component.onCompleted: {
        Logic.updateTaskNames(taskCombo)
        VideoStream.refreshNames(undefined,
                                 (topics) => {
                                     videoStreamCombo.model = topics
                                 })
    }

    Component.onDestruction: {
        if(currentTask.length > 0) {
            settings.currentTask = currentTask
        }
        settings.sync()
    }
}

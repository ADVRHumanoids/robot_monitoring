import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import "Joy.js" as Logic

Card {

    name: 'Joy Setup'
    property alias currentTask: taskCombo.currentText
    property real maxSpeed: maxSpeedLinearSpinBox.value

    // private
    id: root
    configurable: false

    toolButtons: [
        Label {
            text: taskCombo.enabled ? `Current task:  <i>${currentTask}</i>` :
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

        enabled: taskCombo.enabled

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
            Layout.columnSpan: 2
            id: maxSpeedLinearSpinBox
            from: 0.0
            to: 2.0
        }
    }

    Settings {
        id: settings
        category: 'JoyCartesianCard'
        property string currentTask
    }

    Component.onCompleted: {
        Logic.updateTaskNames(taskCombo)
    }

    Component.onDestruction: {
        if(currentTask.length > 0) {
            settings.currentTask = currentTask
        }
        settings.sync()
    }
}

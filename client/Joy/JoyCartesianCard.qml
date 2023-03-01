import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import "../Common"
import "Joy.js" as Logic

Card {

    name: 'Joy Setup'
    property alias currentTask: taskCombo.currentText
    property real maxSpeed: maxSpeedLinearSpinBox.value
    property bool ikRunning: false

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

    Timer {
        repeat: true
        interval: 5000
        onTriggered: Logic.updateTaskNames(taskCombo)
        Component.onCompleted: start()
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

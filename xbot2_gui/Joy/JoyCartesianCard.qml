import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Common
import "Joy.js" as Logic
import "../Video/VideoStream.js" as VideoStream


Card1 {

    name: ikRunning ? `Current task:  <i>${currentTask}</i>` :
                      'No active task'
    property alias currentTask: taskCombo.currentText
    property bool ikRunning: false
    property alias videoStream: videoStreamCombo.currentText

    // private
    id: root
    configurable: false
    collapsed: true

    frontItem: GridLayout {

        id: grid

        anchors.fill: parent

        columns: 3

        columnSpacing: CommonProperties.geom.margins

        Button {
            Layout.columnSpan: 3
            Layout.fillWidth: true
            text: 'Refresh tasks'
            onReleased: {
                Logic.updateTaskNames(taskCombo)
                Logic.taskIsEnabled(taskCombo.currentText,
                                    (is_active) => {
                                        grid.taskActive = is_active
                                    })
            }
        }

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
                console.log(`model ${model}`)
                currentIndex = 0
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
            text: 'Video stream'
        }

        ComboBox {
            id: videoStreamCombo
            Layout.fillWidth: true
            onModelChanged: {
                if(count > 0) currentIndex = 0
            }
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

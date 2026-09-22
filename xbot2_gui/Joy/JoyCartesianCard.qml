import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Common
import "Joy.js" as Logic
import "../VideoMpegTs/VideoStream.js" as VideoStream


Card1 {

    name: ikRunning ? `Current task:  <i>${currentTask}</i>` :
                      'No active task'
    property alias currentTask: taskCombo.currentText
    property bool ikRunning: false
    property alias videoStream: videoStreamCombo.currentText
    property alias videoEnabled: videoEnabledSwitch.checked

    // private
    id: root
    configurable: false
    collapsed: true

    toolButtons: [
        Switch {
            id: videoEnabledSwitch
            text: 'Video'
            checked: false
        }

    ]

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
            id: refreshVideoStreamsBtn
            text: 'Refresh'
            onClicked: {
                let videoSources = videoSourcesInput.values()
                videoStreamCombo.model = []
                VideoStream.refreshNames(videoSources,
                                         (topics) => {
                                             videoStreamCombo.model = videoStreamCombo.model.concat(topics)
                                             videoStreamCombo.currentTextChanged()
                                         })
            }
        }

        Label {
            text: 'Video sources'
        }

        TokenInput {
            id: videoSourcesInput

            Layout.fillWidth: true
            Layout.columnSpan: 2

            placeholderText: "Add source..."

            onTokenAdded: text => console.log("Added:", text)
            onTokenRemoved: text => console.log("Removed:", text)

            Component.onCompleted: {
                let savedSourcesFound = false
                for(const source of settings.videoSources) {
                    savedSourcesFound = true
                    addToken(source)
                }
                if(!savedSourcesFound) {
                    addToken('localhost')
                }
                refreshVideoStreamsBtn.clicked()
            }
        }
    }

    Settings {
        id: settings
        category: 'JoyCartesianCard'
        property string currentTask
        property list<string> videoSources
    }

//    Timer {
//        repeat: true
//        interval: 5000
//        onTriggered: Logic.updateTaskNames(taskCombo)
//        Component.onCompleted: start()
//    }

    Component.onCompleted: {
        Logic.updateTaskNames(taskCombo)
    }

    Component.onDestruction: {
        if(currentTask.length > 0) {
            settings.currentTask = currentTask
        }
        settings.videoSources = videoSourcesInput.values()
        settings.sync()
    }
}

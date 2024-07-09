import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtTextToSpeech

import Main
import Common
import Audio

import "./TransportationTasks" as Tasks
import "Transportation.js" as Logic

Item {

    property ClientEndpoint client
    signal pageSelected()

    // property list<string> commands: ['Teaching', 'Close Gripper', 'Lift', 'Transportation', 'Place', 'Open Gripper']
    property list<string> state_commands: ['Teaching', 'Follow', 'Autonomous']
    property list<bool> active_states
    property list<string> task_commands: ['Close Gripper', 'Open Gripper', 'Lift', 'Place']
    property string stop_command: 'Stop'

    ColumnLayout {

        anchors.left: parent.left
        anchors.top: parent.top

        GridLayout {

            columns: 2
            columnSpacing: 16
            uniformCellHeights: true

            Label {
                text: 'Voice commands'
                font.pixelSize: CommonProperties.font.h3
            }
            Switch {
                id: asrSwitch
                checked: false
                onClicked: Logic.setAsrState(checked)

                Connections {
                    target: AudioBroadcaster
                    function onActiveChanged() {
                        asrSwitch.checked = AudioBroadcaster.active
                    }
                }
            }

            //
            Label {
                text: 'Recognized text'
                font.pixelSize: CommonProperties.font.h3
            }
            Label {
                id: speechTextLabel
                text: '--'
                Layout.fillWidth: true
                font.pixelSize: CommonProperties.font.h3
            }

            //
            Label {
                text: 'Current state'
                font.pixelSize: CommonProperties.font.h3
            }
            Label {
                id: currStateLabel
                text: '--'
                Layout.fillWidth: true
                font.pixelSize: CommonProperties.font.h3
            }

            //
            Label {
                text: 'Current command'
                font.pixelSize: CommonProperties.font.h3
            }
            Label {
                id: currCmdLabel
                text: '--'
                Layout.fillWidth: true
                font.pixelSize: CommonProperties.font.h3
            }
        }

        Item {
            Layout.fillHeight: true
        }

        Item {
            Layout.preferredHeight: 24
        }

    }

    TextToSpeech {
        id: tts
    }

    DelayButton {
        id: stopButton
        text: stop_command
        width: 200
        height: 80
        Layout.alignment:Qt.AlignCenter
        anchors.horizontalCenter: parent.horizontalCenter
        // anchors.verticalCenter: parent.verticalCenter
        anchors.bottom: parent.bottom

        contentItem: Text {
                id: stopName
                text: stop_command
                font.pixelSize: CommonProperties.font.h2
                font.bold: true
                color: "#ee99a0"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
        }

        font.pixelSize: CommonProperties.font.h1
        Layout.fillWidth: true
        Layout.preferredHeight: 80 // 120

        onActivated: {
            console.log("Command: ", text.toLowerCase())
            stopAnimation.start()
            Logic.sendCommand(text.toLowerCase())
            progress = 0
        }

        background: Rectangle {
            id: stopBackground
            color: "#363a4f"
            anchors.centerIn: parent
            width: parent.width
            height: parent.height
            radius: 10
            border.width: 5
            border.color: "#ee99a0"
        }
        ParallelAnimation {
            id: stopAnimation
            SequentialAnimation {
                property int transitionTime: 250
                PropertyAnimation {
                    target: stopBackground
                    property: "color"
                    to: "#ee99a0"
                    duration: 250
                }
                PropertyAnimation {
                    target: stopBackground
                    property: "color"
                    to: "#363a4f"
                    duration: 250
                }
            }

            SequentialAnimation {
                PropertyAnimation {
                    target: stopName
                    property: "color"
                    to: "#363a4f"
                    duration: 250
                }
                PropertyAnimation {
                    target: stopName
                    property: "color"
                    to: "#ee99a0"
                    duration: 250
                }
            }
        }
    }

    RowLayout
    {
        // commands layout
        id: commandsLayout
        Layout.alignment:Qt.AlignCenter
        anchors.horizontalCenter: parent.horizontalCenter
        // anchors.bottom: parent.bottom
        anchors.bottom: stopButton.top
        anchors.bottomMargin: 20
        spacing: 69
        property bool teachingActive: false
        property bool followActive: false
        property bool autonomousActive: false

        ScrollView {
            Layout.fillWidth: true
            // anchors.centerIn: parent
            width: 200
            height: 600
            // ScrollBar.vertical.policy: ScrollBar.AlwaysOn
            contentWidth: availableWidth
            id: scroll

            ColumnLayout {
                spacing: 10
                width: scroll.contentWidth
                id: statesColumn
                Label {
                    Layout.alignment: Qt.AlignCenter
                    text: "States"
                    font.pixelSize: CommonProperties.font.h1
                    font.bold: true
                    color: "#cad3f5"
                    horizontalAlignment: Text.AlignJustify // Center text horizontally
                    verticalAlignment: Text.AlignVCenter
                }

                Repeater {
                    // id: statesButton
                    model: state_commands
                    delegate: DelayButton {
                        text: modelData
                        id: commandButton
                        contentItem: Text {
                                id: stateName
                                text: modelData
                                font.pixelSize: CommonProperties.font.h2
                                font.bold: true
                                color: "#b8c0e0"
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                // elide: Text.ElideRight
                        }
                        font.pixelSize: CommonProperties.font.h1
                        Layout.fillWidth: true
                        Layout.preferredHeight: 80// 120

                        onActivated: {
                            Logic.sendCommand(text.toLowerCase())
                            progress = 0
                        }

                        background: Rectangle {
                            color: "#363a4f"
                            anchors.centerIn: parent
                            width: parent.width
                            height: parent.height
                            radius: 10
                            border.width: 5
                            border.color: "#c6a0f6"
                        }

                        Rectangle {
                            anchors.centerIn: parent
                            width: parent.width
                            height: parent.height
                            radius: 10
                            color: "transparent"
                            opacity: 1.0
                            border.color: "#8bd5ca"
                            border.width: 5
                            visible: active_states[index]// parent.text === currCmdLabel.text
                        }
                    }
                }
            }
        }

        Tasks.TeachingTasks
        {
            id: teachingTasks
            visible: active_states[0]
        }

        Tasks.FollowTasks
        {
            id:followTasks
            visible: active_states[1]
        }

        Tasks.AutonomousTasks
        {
            id: autonomousTasks
            visible: active_states[2]
        }
    }

    Connections {
        target: client

        function onObjectReceived(msg) {

            if(msg.type === 'speech_text') {
                speechTextLabel.text = msg.text
                return
            }

            if(msg.type === 'speech_cmd') {

                if(msg.cmd === '__start__') {
                    currStateLabel.text = 'waiting for command'
                    tts.say(currStateLabel.text)
                }
                else if(msg.cmd === '__invalid__') {
                    currStateLabel.text = 'invalid command'
                    tts.say(currStateLabel.text)
                }
                else if(msg.cmd === '__done__') {
                    tts.say('executing command')
                }
                else if(msg.cmd === '__timeout__') {
                    currStateLabel.text = 'no command received'
                    tts.say(currStateLabel.text)
                }
                else if(msg.cmd === '__error__') {
                    currStateLabel.text = 'an error occurred'
                    currCmdLabel.text += ' (failed)'
                    tts.say(currStateLabel.text)
                }
                else if(msg.cmd === '__end__') {
                }
                else {
                    currStateLabel.text = 'will execute command: ' + msg.cmd
                    currCmdLabel.text = Logic.toTitleCase(msg.cmd)
                    tts.say(currStateLabel.text)
                }              

                return
            }

            if (msg.type === 'coworker_state')
            {

                // close all the task menu
                for (var k in active_states)
                {
                    active_states[k] = false
                }

                switch (msg.state)
                {
                case "teaching":
                    console.log("Openign teching task")
                    active_states[0] = true
                    break
                case "ffm":
                    console.log("Openign ffm task")
                    active_states[1] = true
                    break
                case "autonomous":
                    console.log("Openign autonomous task")
                    active_states[2] = true
                    break
                default:
                    console.warn("Unknown State!")
                    break
                }
                return

            }
        }

    }

    Component.onCompleted: {

        client.doRequestAsync('GET', '/speech/info', '')
        .then((msg) => {
                  state_commands = msg.body.commands.state.map(Logic.toTitleCase)
              })

        for (var i = 0; i < state_commands.length; ++i)
        {
            active_states[i] = false
        }

        Logic.setAsrState(false)
    }


}

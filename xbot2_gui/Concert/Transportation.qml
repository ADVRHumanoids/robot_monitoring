import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtTextToSpeech

import Main
import Common
import Audio

import "Transportation.js" as Logic

Item {

    property ClientEndpoint client

    signal pageSelected()

    property list<string> commands: ['Teaching', 'Close Gripper', 'Lift', 'Transportation', 'Place', 'Open Gripper']

    ColumnLayout {

        anchors.fill: parent

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

        ScrollView {

            Layout.fillWidth: true
            contentWidth: availableWidth
            id: scroll

            GridLayout {
                uniformCellWidths: true
                columns: width / 200
                columnSpacing: 16
                width: scroll.contentWidth
                Repeater {
                    model: commands
                    delegate: DelayButton {
                        text: modelData
                        font.pixelSize: CommonProperties.font.h1
                        Layout.fillWidth: true
                        Layout.preferredHeight: 120

                        onActivated: {
                            Logic.sendCommand(text.toLowerCase())
                            progress = 0
                        }

                        Rectangle {
                            anchors.centerIn: parent
                            width: parent.width + 8
                            height: parent.height
                            color: Qt.alpha(CommonProperties.colors.accent, 0.3)
                            // border.color: CommonProperties.colors.accent
                            // border.width: 2
                            visible: parent.text === currCmdLabel.text
                        }
                    }
                }
            }

        }


        Item {
            Layout.preferredHeight: 24
        }

    }

    TextToSpeech {
        id: tts
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
        }

    }

    onPageSelected: {

        client.doRequestAsync('GET', '/speech/info', '')
        .then((msg) => {
                  commands = msg.body.commands.map(Logic.toTitleCase)
              })

        Logic.setAsrState(false)
    }


}

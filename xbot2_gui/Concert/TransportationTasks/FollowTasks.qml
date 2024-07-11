import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtTextToSpeech

import Main
import Common
import Audio
import Concert

import "../Transportation.js" as Logic

ColumnLayout
{
    property list<string> follow_tasks: ['Place']
    property bool busy: false
    id: followTaskColumn
    spacing: 10
    Layout.minimumWidth: 126
    // width: scrollTask.contentWidth

    Label {
        Layout.alignment: Qt.AlignCenter
        text: "Tasks"
        font.pixelSize: CommonProperties.font.h1
        font.bold: true
        color: "#cad3f5"
        horizontalAlignment: Text.AlignJustify // Center text horizontally
        verticalAlignment: Text.AlignVCenter

    }

    Repeater {
        // id: statesButton
        model: follow_tasks
        delegate: DelayButton {
            text: modelData
            contentItem: Text {
                    id: taskName
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

            PropertyAnimation {
                id: hideText
                // running: parent.text === currCmdLabel.text && busy
                target: taskName
                property: "color"
                to: "transparent"
                duration: 250
            }

            PropertyAnimation {
                id: showText
                target: taskName
                property: "color"
                to: "#b8c0e0"
                duration: 250
            }

            onActivated: {
                Logic.sendCommand(text.toLowerCase())
                hideText.start()
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
                border.color: "#8bd5ca"
                border.width: 5
                visible: parent.text === currCmdLabel.text && busy
            }

            TaskBusyIndicator
            {
                id: teachingBusyIndicator
                visible: parent.text === currCmdLabel.text && busy
                anchors.centerIn: parent
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Connections {
                target: client
                function onObjectReceived(msg) {
                    if (msg.type === 'coworker_is_task_running')
                    {
                        busy = msg.state
                        if (msg.state)
                        {

                        }
                        else
                        {
                            showText.start()
                        }
                    }
                }
            }
        }
    }
    Component.onCompleted: {

        client.doRequestAsync('GET', '/speech/info', '')
        .then((msg) => {
                  var allowedTasks = []
                  var i = 0
                  for (var name in msg.body.allowed_task.follow)
                  {
                      allowedTasks.push(msg.body.allowed_task.follow[i])
                      ++i;
                  }
                  follow_task = allowedTasks.map(Logic.toTitleCase)
              })
    }
}

import QtQuick 2.4
import QtQuick.Layouts 1.3
import QtQuick.Controls 6.4

import "../sharedData.js" as SharedData

Item {

    id: root
    property var client: undefined
    property var items: Object()

    ColumnLayout {


        anchors.fill: parent


        RowLayout {

            id: rowLayout
            width: parent.width

            Repeater {

                id: procRepeater
                model: 0

                Process {

                    Layout.fillWidth: true
                    name: modelData.name
                    status: modelData.status

                    onStart: {

                        root.client.doRequest('PUT',
                                              '/proc',
                                              JSON.stringify({name: name, cmd: 'start'}),
                                              function(msg){})
                    }

                    onStop: {

                        root.client.doRequest('PUT',
                                              '/proc',
                                              JSON.stringify({name: name, cmd: 'stop'}),
                                              function(msg){})
                    }

                    configureBtn.onPressed: {
                        root.spawnConfigurePanel(index)
                    }


                }

            }

        }

        ToolSeparator {
            Layout.fillWidth: true
            orientation: Qt.Horizontal
        }

        RowLayout {
            CheckBox {
                id: scrollOnOutputCheck
                text: "Scroll on output"
                checked: true
                checkable: true
            }
            Button {
                id: clearBtn
                text: "Clear console"
                onReleased: {
                    consoleText.clear()
                }
            }
        }

        ScrollView {

            Layout.fillHeight: true
            Layout.fillWidth: true

            TextArea {

                width: parent.width

                id: consoleText
                color: "white"
                readOnly: true

                placeholderText: "Console output"
                wrapMode: TextEdit.Wrap

                textFormat: TextEdit.RichText

                function addText(str) {
                    append(str)
                    if(scrollOnOutputCheck.checked) {
                        cursorPosition = length - 1
                    }
                }

            }

        }

    }

    ConfigurePanel {
        id: configurePanel
    }

    function handleProcMessage(msg) {

        // look for process with name msg.name
        for(let i = 0; i < procRepeater.count; i++) {

            var item_i = procRepeater.itemAt(i)

            // found!
            if(item_i.name === msg.name) {

                if(msg.content === 'status')
                {
                    item_i.status = msg.status
                }

                if(msg.content === 'output')
                {
                    if(msg.stdout.length > 0) {
                        consoleText.addText(msg.stdout)
                    }

                    if(msg.stderr.length > 0) {
                        consoleText.addText('<font color="red">' + msg.stderr + '</>')
                    }
                }

                break
            }
        }
    }

    function spawnConfigurePanel(index) {
        configurePanel.description = SharedData.processInfo[index].cmdline
        configurePanel.hidden = false
    }

    function finalize() {
        client.procMessageReceived.connect(handleProcMessage)
    }

    Component.onCompleted: {
        console.log('constructing Console')
        console.log(JSON.stringify(SharedData.processInfo))
        procRepeater.model = SharedData.processInfo
    }

}

import QtQuick 2.4
import QtQuick.Layouts 1.3
import QtQuick.Controls 6.4

import "../sharedData.js" as SharedData

Item {

    id: root
    clip: true

    // set this to the main ClientEndpoint object
    property var client: undefined

    // vertically stack process row button and console
    ColumnLayout {

        anchors.fill: parent

        // process widget row
        RowLayout {

            id: rowLayout
            width: parent.width

            Repeater {

                id: procRepeater
                model: 0

                Process {

                    id: process

                    ConfigurePanel {
                        id: configurePanel
                        z: 10
                        parent: root
                        description: SharedData.processInfo[index].cmdline
                    }

                    Layout.fillWidth: true
                    name: modelData.name
                    status: modelData.status

                    onStart: {

                        root.client.doRequest('PUT',
                                              '/proc',
                                              JSON.stringify({name: name,
                                                              cmd: 'start',
                                                              options: configurePanel.options}),
                                              function(msg){})
                    }

                    onStop: {

                        root.client.doRequest('PUT',
                                              '/proc',
                                              JSON.stringify({name: name, cmd: 'stop'}),
                                              function(msg){})
                    }

                    configureBtn.onPressed: {
                        if(configPanelOpen) {
                             configurePanel.hidden = true
                        }
                        else {
                            configurePanel.hidden = false
                        }


                    }

                    configPanelOpen: !configurePanel.hidden


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
                    let prefix = '[' + item_i.name + '] '
                    if(msg.stdout.length > 0) {
                        consoleText.addText(prefix+msg.stdout)
                    }

                    if(msg.stderr.length > 0) {
                        consoleText.addText('<font color="red">' + prefix+msg.stderr + '</>')
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

    onClientChanged:  {
        client.procMessageReceived.connect(handleProcMessage)
    }

    Component.onCompleted: {
        procRepeater.model = SharedData.processInfo
    }

}

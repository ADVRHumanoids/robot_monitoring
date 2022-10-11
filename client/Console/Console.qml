import QtQuick 2.4
import QtQuick.Layouts 1.3
import QtQuick.Controls 6.4

import "../sharedData.js" as SharedData

Item {

    id: root
    property var client: undefined
    property var items: Object()

    Rectangle {

        property bool hidden: true

        id: configureArea
        height: parent.height
        width: 300
        x: hidden ? parent.width : parent.width - width
        color: Qt.rgba(1, 0, 0, 0.3)
        z: 1


        Behavior on x {
            NumberAnimation {
                duration: 500
                easing.type: Easing.OutQuad
            }
        }

        MouseArea {
            anchors.right: parent.left
            enabled: !parent.hidden
            height: parent.height
            width: 200

            onReleased: {
                parent.hidden = true
            }
        }
    }



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
                    cmdList: modelData.cmdList

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
                        console.log('configure')
                        configureArea.hidden = false
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

        function handleProcMessage(msg) {
            for(let i = 0; i < procRepeater.count; i++) {
                var item_i = procRepeater.itemAt(i)
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

        function finalize() {
            client.procMessageReceived.connect(handleProcMessage)
        }

        Component.onCompleted: {
            console.log('constructing Console')
            console.log(JSON.stringify(SharedData.processInfo))
            procRepeater.model = SharedData.processInfo
        }

    }

}

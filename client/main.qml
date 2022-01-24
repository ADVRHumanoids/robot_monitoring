import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts
import QtQuick.Controls

import "SingleJointState"

Window {
    id: mainWindow
    width: 360
    height: 640
    visible: true
    title: qsTr("Xbot2 Robot GUI")

    StackLayout {

        id: stack
        anchors.fill: parent

        HelloScreen {
            id: hello
            onUpdateServerUrl: function(host, port){
                client.active = false
                client.url = "ws://" + host + ":" + port + "/ws"
                client.active = true
            }
        }

        ColumnLayout {

            anchors.fill: parent

            RowLayout {

                Label {
                    text: "Select joint"
                }

                Item {
                    Layout.fillWidth: true
                }

                ComboBox {
                    id: jointCombo
                    editable: true
                    wheelEnabled: true
                    Layout.preferredWidth: 200
                }
            }

            SingleJointStateStack {
                id: singleJointState
                Layout.fillHeight: true
                Layout.fillWidth: true
                currentIndex: jointCombo.currentIndex
            }
        }



    }



    ClientEndpoint {
        id: client
        onError: function (msg) {
            hello.setError(msg)
        }
        onConnected: function (msg) {
            hello.setConnected(msg)
        }
    }

    function constructJointStateItem (js_msg) {
        singleJointState.construct(js_msg.name)
        jointCombo.model = js_msg.name
        client.jointStateReceived.disconnect(constructJointStateItem)
        stack.currentIndex = 1
    }

    Component.onCompleted: {
        client.jointStateReceived.connect(constructJointStateItem)
        client.jointStateReceived.connect(singleJointState.setJointStateMessage)
    }


}

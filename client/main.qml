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
    title: "Xbot2 Robot GUI"

    onWidthChanged: {
        scroll.requiredContentWidth = Math.max(320, width)
    }

    onHeightChanged: {
        scroll.requiredContentHeight = Math.max(568, height)
    }


    ScrollView
    {
        id: scroll
        anchors.fill: parent
        clip: true

        property int requiredContentWidth: mainWindow.width
        property int requiredContentHeight: mainWindow.height

        contentWidth: requiredContentWidth
        contentHeight: requiredContentHeight

        StackLayout {

            id: stack
            anchors.fill: parent

            HelloScreen {

                id: hello
                Layout.fillWidth: true
                Layout.fillHeight: true

                onUpdateServerUrl: function(host, port){
                    client.active = false
                    client.url = "ws://" + host + ":" + port + "/ws"
                    client.active = true
                }
            }

            ColumnLayout {

                Layout.fillWidth: true
                Layout.fillHeight: true

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

                    onConstructionCompleted: {
                        stack.currentIndex = 1
                    }

                    onProgressChanged: function (msg) {
                        hello.setProgress('constructing UI, this could take a while...\n' + msg)
                    }

                }
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
        client.jointStateReceived.disconnect(constructJointStateItem)
        hello.setProgress('constructing UI, this could take a while..')
        singleJointState.construct(js_msg.name)
        jointCombo.model = js_msg.name
    }

    Component.onCompleted: {
        client.jointStateReceived.connect(constructJointStateItem)
        client.jointStateReceived.connect(singleJointState.setJointStateMessage)
    }

}

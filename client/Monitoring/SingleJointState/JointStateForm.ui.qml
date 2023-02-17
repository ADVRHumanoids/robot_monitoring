import QtQuick 2.4

Item {
    width: 400
    height: 400

    onWidthChanged: {
        scroll.requiredContentWidth = Math.max(300, width)
    }

    onHeightChanged: {
        scroll.requiredContentHeight = Math.max(520, height)
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
}

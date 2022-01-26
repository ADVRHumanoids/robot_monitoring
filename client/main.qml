import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import QtQuick.Shapes 1.14

import "SingleJointState"
import "BarPlot"

Window {

    id: mainWindow
    width: 360
    height: 568
    visible: true

    GridLayout
    {
        anchors.fill: parent
        columns: 2
        columnSpacing: 10
        rowSpacing: 2
        anchors.margins: 8

        Label {
            text: "joint1"
        }

        TwoSideBar {
            Layout.fillWidth: true
            min: 0
            max: 2
            value: 1.5
        }

        Label {
            text: "joint1"
        }

        TwoSideBar {
            Layout.fillWidth: true
            min: -2
            max: 2
            value: 1.5
        }

        Label {
            text: "joint1"
        }

        TwoSideBar {
            Layout.fillWidth: true
            min: -2
            max: 2
            value: 0.5
        }

        Label {
            text: "joint1"
        }

        TwoSideBar {
            Layout.fillWidth: true
            min: -2
            max: 2
            value: -1.5
        }

        Label {
            text: "joint1"
        }

        TwoSideBar {
            Layout.fillWidth: true
            min: -2
            max: 2
            value: 1.5
        }

        Label {
            text: "joint1"
        }

        TwoSideBar {
            Layout.fillWidth: true
            min: -2
            max: 2
            value: 0.5
        }

    }

}

//Window {

//    id: mainWindow
//    width: 360
//    height: 568
//    visible: true
//    title: "Xbot2 Robot GUI"

//    onWidthChanged: {
//        scroll.requiredContentWidth = Math.max(300, width)
//    }

//    onHeightChanged: {
//        scroll.requiredContentHeight = Math.max(520, height)
//    }


//    ScrollView
//    {
//        id: scroll
//        anchors.fill: parent
//        clip: true

//        property int requiredContentWidth: mainWindow.width
//        property int requiredContentHeight: mainWindow.height

//        contentWidth: requiredContentWidth
//        contentHeight: requiredContentHeight

//        StackLayout {

//            id: stack
//            anchors.fill: parent

//            HelloScreen {

//                id: hello
//                Layout.fillWidth: true
//                Layout.fillHeight: true

//                onUpdateServerUrl: function(host, port){
//                    client.active = false
//                    client.url = "ws://" + host + ":" + port + "/ws"
//                    client.active = true
//                }
//            }

//            ColumnLayout {

//                Layout.fillWidth: true
//                Layout.fillHeight: true

//                RowLayout {

//                    Label {
//                        text: "Select joint"
//                    }

//                    Item {
//                        Layout.fillWidth: true
//                    }

//                    ComboBox {
//                        id: jointCombo
//                        editable: true
//                        wheelEnabled: true
//                        Layout.preferredWidth: 200
//                    }
//                }

//                SingleJointStateStack {
//                    id: singleJointState
//                    Layout.fillHeight: true
//                    Layout.fillWidth: true
//                    currentIndex: jointCombo.currentIndex

//                    onConstructionCompleted: {
//                        stack.currentIndex = 1
//                    }

//                    onProgressChanged: function (msg) {
//                        hello.setProgress('constructing UI, this could take a while...\n' + msg)
//                    }

//                }
//            }
//        }
//    }



//    ClientEndpoint {
//        id: client
//        onError: function (msg) {
//            hello.setError(msg)
//        }
//        onConnected: function (msg) {
//            hello.setConnected(msg)
//        }
//    }

//    function constructJointStateItem (js_msg) {
//        client.jointStateReceived.disconnect(constructJointStateItem)
//        hello.setProgress('constructing UI, this could take a while..')
//        singleJointState.construct(js_msg.name)
//        jointCombo.model = js_msg.name
//    }

//    Component.onCompleted: {
//        client.jointStateReceived.connect(constructJointStateItem)
//        client.jointStateReceived.connect(singleJointState.setJointStateMessage)
//    }

//}

import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import QtQuick.Shapes 1.14

import "SingleJointState"
import "BarPlot"
import "sharedData.js" as SharedData
import "Plotter"
import "Menu"

ApplicationWindow {

    id: mainWindow
    width: 1024
    height: 768
    visible: true
    title: "Xbot2 Robot GUI"

    property var items: Object()

    ListModel {

        id: pagesModel

        ListElement {
            name: "Home"
            page: "HelloScreen.qml"
        }

        ListElement {
            name: "Monitoring"
            page: "Monitoring.qml"
        }

    }

    SlidingMenu {
        id: menu
        z: 1
        anchors.fill: parent
        model: pagesModel

        onItemSelected: function(index) {
            pagesStack.currentIndex = index
            closeMenu()
        }
    }

    StackLayout {

        id: pagesStack
        anchors.fill: parent

        Repeater {

            model: pagesModel

            Loader {

                Layout.fillHeight: true
                Layout.fillWidth: true

                active: pagesStack.currentIndex === index
                source: page

                onLoaded: {
                    items[name.toLowerCase()] = item
                    active = true
                }

                Connections {

                    target: item

                    // connect item to client update function
                    function onUpdateServerUrl(hostname, port) {
                        client.hostname = hostname
                        client.port = port
                        client.active = true
                    }
                }
            }

        }

    }


    ClientEndpoint {
        id: client
        onError: function (msg) {
            items.home.setError(msg)
        }
        onConnected: function (msg) {
            items.home.setConnected(msg)
        }
        onJointStateReceived: function (msg) {
            if(items.monitoring !== undefined) {
                items.monitoring.setJointStateMessage(msg)
            }
        }
        onFinalized: {
            print('finalized!')
            pagesStack.currentIndex = 1
        }
    }

    Timer {
        id: reconnectTimer
        interval: 1000
        running: true
        repeat: true

        onTriggered: {
            client.active = true
        }

    }
}

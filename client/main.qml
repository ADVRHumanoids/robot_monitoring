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
import "Console"

ApplicationWindow {

    id: mainWindow
    width: 1024
    height: 768
    visible: true
    title: "Xbot2 Robot GUI"

    property var items: Object()

    // this model contains all main pages
    // (i) hello page (select server address)
    // (ii) monitoring page
    ListModel {

        id: pagesModel

        // the hello page
        ListElement {
            name: "Home"
            page: "HelloScreen.qml"
        }

        // the console page
        ListElement {
            name: "Console"
            page: "Console/Console.qml"
        }

        // the monitoring page
        ListElement {
            name: "Monitoring"
            page: "Monitoring.qml"
        }

    }

    // a sliding menu to select the active page
    SlidingMenu {
        id: menu
        z: 1
        anchors.fill: parent
        model: pagesModel

        // when a page is selected, make it active
        // on the stack layout
        onItemSelected: function(index) {
            pagesStack.currentIndex = index
            closeMenu()
        }
    }

    // stack with all main pages, as defined
    // in the pagesModel element
    StackLayout {

        id: pagesStack
        anchors.fill: parent

        // load all pages in the model
        Repeater {

            model: pagesModel

            // lazy-loading of active page
            Loader {

                Layout.fillHeight: true
                Layout.fillWidth: true

                active: pagesStack.currentIndex === index
                source: page

                onLoaded: {
                    items[name.toLowerCase()] = item
                    active = true
                    item.client = client
                    item.finalize()
                }

                Connections {

                    target: item  // item is the loaded object

                    // connect hello page to client update function
                    function onUpdateServerUrl(hostname, port) {
                        client.hostname = hostname
                        client.port = port
                        client.active = true
                    }
                }
            }
        }
    }

    // the object handling communication with the backend (server)
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
            pagesStack.currentIndex = 1  // show monitoring page
        }
    }

    // a timer to periodically try connection
    // with server
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

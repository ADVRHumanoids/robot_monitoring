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
import "Cartesian"
import "TestThings"

ApplicationWindow {

    id: mainWindow
    width: 1280
    height: 720
    visible: true
    title: "Xbot2 Robot GUI"

    property var items: Object()

    // this model contains all main pages
    // (i) hello page (select server address)
    // (ii) monitoring page
    ListModel {

        id: pagesModel

        ListElement {
            name: "Test Page"
            page: "TestThings/TestPage.qml"
            requirement: "none"
        }

        // the hello page
        ListElement {
            name: "Home"
            page: "HelloScreen.qml"
            requirement: "none"
        }

        // the console page
        ListElement {
            name: "Console"
            page: "Console/Xbot2.qml"
            requirement: "active"  // server connected
        }

        // the monitoring page
        ListElement {
            name: "Monitoring"
            page: "Monitoring.qml"
            requirement: "finalized"  // xbot2 connected
        }

        // the cartesian control page
        ListElement {
            name: "Cartesian control"
            page: "Cartesian/Cartesian.qml"
            requirement: "finalized"  // xbot2 connected
        }
    }


//    Rectangle {
//        id: cose
//        height: parent.height
//        width: 90
//        anchors.left: parent.left
//        color: "red"
//        z: 1

//        // a sliding menu to select the active page
//        SlidingMenu {

//            id: menu

//            width: mainWindow.width
//            height: mainWindow.height
//            model: pagesModel

//            handleWidth: parent.width

//            entryActiveCallback: function(i) {
//                if(pagesModel.get(i).requirement === "none") {
//                    return true
//                }
//                if(pagesModel.get(i).requirement === "active") {
//                    return client.active
//                }
//                if(pagesModel.get(i).requirement === "finalized") {
//                    return client.isFinalized
//                }
//            }

//            // when a page is selected, make it active
//            // on the stack layout
//            onItemSelected: function(index) {
//                pagesStack.currentIndex = index
//                closeMenu()
//            }
//        }

//    }

    NavDrawer {
        id: nav
        anchors.left: parent.left
        height: parent.height
        z: 1
        model: pagesModel
        overlayWidth: mainWindow.width
    }

    // stack with all main pages, as defined
    // in the pagesModel element
    StackLayout {

        id: pagesStack

        anchors {
            left: nav.right
            right: parent.right
        }

        height: mainWindow.height


        // load all pages in the model
        Repeater {

            model: pagesModel

            // lazy-loading of active page
            Loader {

                Layout.fillHeight: true
                Layout.fillWidth: true

                active: pagesStack.currentIndex === index

                onLoaded: {
                    items[name.toLowerCase()] = item
                    active = true
                }

                Component.onCompleted: {
                    // this is the "constructor"
                    // each page has a .client elem
                    setSource(page, {'client': client})
                }
            }
        }
    }

    // the object handling communication with the backend (server)
    Item {
        id: client
        property bool active: true
    }

//    ClientEndpoint {
//        id: client
//        onError: function (msg) {
//            items.home.setError(msg)
//        }
//        onConnected: function (msg) {
//            items.home.setConnected(msg)
//            menu.evalActiveEntries()
//        }
//        onJointStateReceived: function (msg) {
//            if(items.monitoring !== undefined) {
//                items.monitoring.setJointStateMessage(msg)
//            }
//        }
//        onFinalized: {
//            print('finalized!')
//            menu.evalActiveEntries()
//        }
//    }

    // a timer to periodically try connection
    // with server
    Timer {
        id: reconnectTimer
        interval: 1000
        running: !client.active
        repeat: true

        onTriggered: {
            client.active = true
        }

    }
}

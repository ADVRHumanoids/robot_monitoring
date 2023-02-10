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
    Item {

        id: pagesModel

        PageItem {
            name: "Home"
            page: "HelloScreen.qml"
            active: true
        }

        PageItem {
            name: "Launcher"
            page: "TestThings/Launcher.qml"
            active: client.active
        }

        PageItem {
            name: "Monitoring"
            page: "TestThings/Monitoring.qml"
            active: client.isFinalized
        }

        PageItem {
            name: "Joy"
            page: "TestThings/Joy.qml"
            active: client.active
        }

        PageItem {
            name: "Viewer"
            page: "TestThings/Viewer3D.qml"
            active: true
        }

        PageItem {
            name: "Playground"
            page: "TestThings/Playground.qml"
            active: true
        }

    }


    NavDrawer {
        id: nav
        anchors.fill: parent
        z: 1
        model: pagesModel

        onCurrentIndexChanged: {
            pagesStack.currentIndex = currentIndex
        }

    }

    NavBar {
        id: navBar
        width: parent.width
        height: 50
        model: pagesModel
        color: nav.color

        onHamburgerClicked: {
            nav.open()
        }

        onCurrentIndexChanged: {
            pagesStack.currentIndex = currentIndex
        }
    }

    function responsiveNav() {
        if(mainWindow.width > mainWindow.height) {
            // landscape
            navBar.y = mainWindow.height
            navBar.visible = false
            nav.railWidth = 72
            nav.menuWidth = 300
        }
        else {
            // portrait
            navBar.y = mainWindow.height - navBar.height
            navBar.visible = true
            nav.railWidth = 0
            nav.menuWidth = mainWindow.width
        }
    }


    // stack with all main pages, as defined
    // in the pagesModel element
    StackLayout {

        id: pagesStack

        anchors {
            right: parent.right
            top: parent.top
            bottom: navBar.top
        }

        width: mainWindow.width - nav.railWidth

        currentIndex: nav.currentIndex

        // load all pages in the model
        Repeater {

            id: pagesStackRepeater

            function reloadAll() {
                console.log(count)
                for(let i = 0; i < count; i++) {
                    let ch = itemAt(i)
                    if(ch.active) {
                        console.log('restarted child ' + ch.pageName)
                        ch.active = false
                        ch.active = true
                    }
                }
            }

            model: pagesModel.children

            // lazy-loading of active page
            Loader {

                id: stackPageLoader
                property string pageName: ''

                Layout.fillHeight: true
                Layout.fillWidth: true

                active: pagesStack.currentIndex === index

                onLoaded: {
                    items[modelData.name.toLowerCase()] = item
                    active = true
                    pageName = modelData.name
                }

                Component.onCompleted: {
                    // this is the "constructor"
                    // each page has a .client elem
                    setSource(modelData.page, {'client': client})
                }

                Connections {
                    target: stackPageLoader.item
                    ignoreUnknownSignals: true
                    function onRestartUi() {
                        console.log('onRestartUi: calling pagesStackRepeater.reloadAll()')
                        pagesStackRepeater.reloadAll()
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
        onFinalized: {
            print('finalized!')
        }
    }

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

    onWidthChanged: {
        responsiveNav()
    }

    onHeightChanged: {
        responsiveNav()
    }

    Component.onCompleted: {
        responsiveNav()
    }
}

import QtQuick
import QtQuick.Window
import QtQuick.Layouts
import QtQuick.Controls

import Common
import Menu
import Font

ApplicationWindow {

    id: mainWindow
    width: 1280
    height: 720
    visible: true
    title: "Xbot2 Robot GUI"
//    visibility: Window.FullScreen

    palette {
        active {
            highlight: Material.primary
            accent: Material.accent
            window: Material.background
        }
    }

    MaterialSymbols {
        id: syms
    }

    property var items: Object()

    Binding {
        target: CommonProperties.geom
        property: 'compactLayout'
        value: mainWindow.width < 600
    }

    Binding {
        target: CommonProperties.geom
        property: 'mediumLayout'
        value: mainWindow.width < 840 && mainWindow.width >= 600
    }

    Binding {
        target: CommonProperties.geom
        property: 'expandedLayout'
        value: mainWindow.width >= 840
    }

    // this model contains all main pages
    // (i) hello page (select server address)
    // (ii) monitoring page
    Item {

        id: pagesModel


        PageItem {
            name: "Playground"
            page: "/qt/qml/TestThings/Playground.qml"
            iconText: syms.playground
            iconFont: syms.font.family
            active: true
        }

        PageItem {
            name: "Home"
            page: "/qt/qml/Home/HelloScreen.qml"
            iconText: syms.home
            iconFont: syms.font.family
            active: true
        }

        PageItem {
            name: "Launcher"
            page: "/qt/qml/Launcher/Launcher.qml"
            iconText: syms.terminal
            iconFont: syms.font.family
            active: client.active
        }

        PageItem {
            name: "Monitoring"
            page: "Monitoring/Monitoring.qml"
            iconText: syms.plot
            iconFont: syms.font.family
            active: client.isFinalized
        }

        PageItem {
            name: "Joy"
            page: "Joy/Joy.qml"
            iconText: syms.joystick
            iconFont: syms.font.family
            active: client.active
        }

        PageItem {
            name: "Viewer"
            page: "Viewer3D/Viewer3D.qml"
            active: true
        }


        PageItem {
            name: "Concert"
            page: "TestThings/ConcertGuidedDrilling.qml"
            active: true
        }

    }


    NavRailWrapper {
        id: nav
        anchors {
            left: parent.left
            top: parent.top
            bottom: parent.bottom
            margins: 8
        }

        width: 80

        z: 1
        model: pagesModel

        visible: CommonProperties.geom.expandedLayout

        onCurrentIndexChanged: {
            pagesStack.currentIndex = currentIndex
            navBar.currentIndex = currentIndex
        }


    }


    NavBarWrapper {

        z: 1

        opacity: 0.8

        visible: !CommonProperties.geom.expandedLayout

        id: navBar
        model: pagesModel

        onCurrentIndexChanged: {
            pagesStack.currentIndex = currentIndex
            nav.currentIndex = currentIndex
        }

        anchors {
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            margins: 8
            leftMargin: CommonProperties.geom.margins
            rightMargin: CommonProperties.geom.margins
        }
    }

    // stack with all main pages, as defined
    // in the pagesModel element
    StackLayout {

        id: pagesStack

        anchors {
            right: parent.right
            top: parent.top
            bottom: navBar.visible ? navBar.top : parent.bottom
            left: nav.visible ? nav.right : parent.left
            margins: 8
            leftMargin: CommonProperties.geom.margins
            rightMargin: CommonProperties.geom.margins
        }

        clip: true

//        width: mainWindow.width - nav.railWidth

        currentIndex: nav.currentIndex

        // load all pages in the model
        Repeater {

            id: pagesStackRepeater

            function reloadAll() {
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
//    Timer {
//        id: reconnectTimer
//        interval: 1000
//        running: !client.active
//        repeat: true

//        onTriggered: {
//            client.active = true
//        }

//    }

}

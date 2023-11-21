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
            buttonText: Material.foreground
            text: Material.foreground
            accent: Material.accent
            window: Material.background
        }
    }

    MaterialSymbols {
        id: syms
    }

    property var items: Object()

    LayoutClassHelper {
        id: layout
        targetWidth: mainWindow.width
    }

    // this model contains all main pages
    // (i) hello page (select server address)
    // (ii) monitoring page
    Item {

        id: pagesModel


        PageItem {
            name: "Playground"
            page: "/qt/qml/TestThings/Playground.qml"
            iconText: MaterialSymbolNames.playground
            iconFont: syms.font.family
            active: true
        }

        PageItem {
            name: "Home"
            page: "/qt/qml/Home/HelloScreen.qml"
            iconText: MaterialSymbolNames.home
            iconFont: syms.font.family
            active: true
        }

        PageItem {
            name: "Process"
            page: "/qt/qml/Launcher/Launcher.qml"
            iconText: MaterialSymbolNames.terminal
            iconFont: syms.font.family
            active: client.isConnected
        }

        PageItem {
            name: "Monitoring"
            page: "/qt/qml/Monitoring/Monitoring.qml"
            iconText: MaterialSymbolNames.plot
            iconFont: syms.font.family
            active: client.isFinalized
        }

        PageItem {
            name: "Joy"
            page: "/qt/qml/Joy/Joy.qml"
            iconText: MaterialSymbolNames.joystick
            iconFont: syms.font.family
            active: client.isConnected
        }

        PageItem {
            name: "Plot"
            page: "/qt/qml/LivePlot/Plot.qml"
            iconText: MaterialSymbolNames.tableChart
            iconFont: syms.font.family
            active: client.isConnected
        }

//        PageItem {
//            name: "Viewer"
//            page: "Viewer3D/Viewer3D.qml"
//            active: true
//        }


//        PageItem {
//            name: "Concert"
//            page: "TestThings/ConcertGuidedDrilling.qml"
//            active: true
//        }

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

        visible: layout.expanded

        onCurrentIndexChanged: {
            pagesStack.currentIndex = currentIndex
            navBar.currentIndex = currentIndex
        }


    }


    NavRailWrapper {

        id: navBar

        visible: !layout.expanded

        model: pagesModel

        orientation: Qt.Horizontal

        uncheckedDisplayMode: AbstractButton.IconOnly
        checkedDisplayMode: AbstractButton.TextBesideIcon

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

        onCurrentIndexChanged: {
            itemAt(currentIndex).item.numErrors = 0
            nav.setBadgeNumber(currentIndex, 0)
        }

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

                active: pagesStack.currentIndex === index || modelData.name === 'Home'

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

                    function onNumErrorsChanged() {
                        if(index !== pagesStack.currentIndex) {
                            nav.setBadgeNumber(index, stackPageLoader.item.numErrors)
                        }
                        else {
                            item.numErrors = 0
                        }
                    }
                }


            }
        }
    }


    NotificationPopup {

        id: notificationPopup
        property bool showPopup: false

        opacity: 0.8

        width: layout.compact ? pagesStack.width : 600
        height: Math.min(implicitHeight, mainWindow.height/2)



        anchors.horizontalCenter: parent.horizontalCenter
        y: showPopup ? mainWindow.height - height - 24 : mainWindow.height

        Behavior on y {
            SequentialAnimation {
                NumberAnimation {
                    easing.type: Easing.OutQuad
                }
                ScriptAction {
                    script: {
                        if(!notificationPopup.showPopup) {
                            notificationPopup.clear()
                        }
                    }
                }
            }
        }

        onShowPopupChanged: {
            if(showPopup) hideTimer.running = true
        }

        Timer {
            id: hideTimer
            interval: 3000
            onTriggered: {
                parent.showPopup = false
            }
        }


        onDismissRequested: {
            showPopup = false
        }

        Connections {
            target: CommonProperties.notifications

            function onNewWarning(txt, name) {
                notificationPopup.addMsg(txt, name, 1)
                notificationPopup.showPopup = true
                hideTimer.restart()
            }

            function onNewError(txt, name) {
                notificationPopup.addMsg(txt, name, 2)
                notificationPopup.showPopup = true
                hideTimer.restart()
            }
        }

    }

    // the object handling communication with the backend (server)
    ClientEndpoint {
        id: client

        onObjectReceived: function(obj) {
            if(obj.type === 'server_log') {
                CommonProperties.notifications.message(obj.txt, 'webserver', obj.severity)
            }
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

import QtQuick
import QtCore
import QtQuick.Window
import QtQuick.Layouts
import QtQuick.Controls

import Common
import Menu
import Font
import Audio

ApplicationWindow {

    id: mainWindow
    width: 1280
    height: 720
    visible: true
    title: "Xbot2 Robot GUI"
    visibility: Qt.platform.os === "android" ? Window.FullScreen : Window.AutomaticVisibility

    property bool dbg: false

    palette {
        active {
            highlight: Material.primary
            highlightedText: Material.foreground
            buttonText: Material.foreground
            text: Material.foreground
            accent: Material.accent
            window: Material.background
        }
        inactive{
            highlight: Material.primary
            highlightedText: Material.foreground
            buttonText: Material.foreground
            text: Material.foreground
            accent: Material.accent
            window: Material.background
        }
        disabled {
            window: Qt.lighter(Material.background)
            buttonText: Material.foreground
        }
    }

    Component.onCompleted: {
        console.log(`palette.disabled.buttonText ${palette.disabled.buttonText.a}`)
    }

    MaterialSymbols {
        id: syms
    }

    property var items: Object()

    LayoutClassHelper {
        id: layout
        targetWidth: mainWindow.width
    }

    Binding {
        target: CommonProperties.geom
        property: 'compactLayout'
        value: layout.compact
    }

    // this model contains all main pages
    // (i) hello page (select server address)
    // (ii) monitoring page
    property list<string> requestedPages
    Item {

        id: pagesModel

        PageItem {
            name: "Home"
            page: "/qt/qml/Home/HelloScreen.qml"
            iconText: MaterialSymbolNames.home
            iconFont: syms.font.family
            active: true
        }

        PageItem {
            name: "Dashboard"
            page: "/qt/qml/Launcher/Dashboard.qml"
            iconText: MaterialSymbolNames.dashboard
            iconFont: syms.font.family
            active: true
        }

        PageItem {
            name: "Process"
            page: "/qt/qml/Launcher/Launcher.qml"
            iconText: MaterialSymbolNames.terminal
            iconFont: syms.font.family
            active: client.isConnected || mainWindow.dbg
        }

        PageItem {
            name: "Monitoring"
            page: "/qt/qml/Monitoring/Monitoring.qml"
            iconText: MaterialSymbolNames.gauge
            iconFont: syms.font.family
            active: client.robotConnected || mainWindow.dbg
            lazyLoad: false
        }

        PageItem {
            name: "Plot"
            page: "/qt/qml/LivePlot/Plot.qml"
            iconText: MaterialSymbolNames.tableChart
            iconFont: syms.font.family
            active: client.isConnected || mainWindow.dbg
        }

        PageItem {
            name: "Joy"
            page: "/qt/qml/Joy/Joy.qml"
            iconText: MaterialSymbolNames.joystick
            iconFont: syms.font.family
            active: client.robotConnected || mainWindow.dbg
        }

        PageItem {
            name: "Horizon"
            page: "/qt/qml/Horizon/Horizon.qml"
            iconText: MaterialSymbolNames.walker
            iconFont: syms.font.family
            active: client.robotConnected || mainWindow.dbg || true
            sizeFactor: 1.1
            visible: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Playground"
            page: "/qt/qml/TestThings/Playground.qml"
            iconText: MaterialSymbolNames.playground
            iconFont: syms.font.family
            active: true
            visible: true
        }

        PageItem {
            name: "Builder"
            page: "/qt/qml/TestThings/Linfa.qml"
            iconText: MaterialSymbolNames.tools
            iconFont: syms.font.family
            active: true
            visible: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Linfa"
            page: "/qt/qml/TestThings/Linfa.qml"
            iconSource: '/Icons/icons/alberobotics100x100.png'
            active: true
            visible: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Drill Task"
            page: "/qt/qml/Concert/Drilling.qml"
            iconText: MaterialSymbolNames.drill
            iconFont: syms.font.family
            active: true
            sizeFactor: 1.2
            visible: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Conda"
            page: "/qt/qml/Concert/Conda.qml"
            iconText: MaterialSymbolNames.wrench
            iconFont: syms.font.family
            active: true
            sizeFactor: 1.1
            visible: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Sanding"
            page: "/qt/qml/Concert/Sanding.qml"
            iconSource: '/Icons/icons/brick_wall_white.png'
            active: true
            sizeFactor: 1.
            visible: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Transportation"
            page: "/qt/qml/Concert/Transportation.qml"
            iconText: MaterialSymbolNames.weight
            iconFont: syms.font.family
            active: true
            visible: requestedPages.indexOf(name) > -1 
        }

        PageItem {
            name: "Ecat"
            page: "/qt/qml/Ecat/Ecat.qml"
            iconText: MaterialSymbolNames.wrench
            iconFont: syms.font.family
            active: true
            sizeFactor: 1.1
            visible: requestedPages.indexOf(name) > -1
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

        sizeFactor: 1.25

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

        currentIndex: nav.currentIndex

        onCurrentIndexChanged: {


            try {
                itemAt(currentIndex).item.numErrors = 0
            }catch(err){}

            try {
                itemAt(currentIndex).item.pageSelected()
            }catch(err){}

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

                active: pagesStack.currentIndex === index

                onLoaded: {

                    console.log(`${modelData.name} loaded`)

                    active = true

                    items[modelData.name.toLowerCase()] = item

                    try {
                        item.pageSelected()
                    }
                    catch(err){}

                    item.pageName = modelData.name

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

    // Rectangle {
    //     color: 'red'
    //     width: 300
    //     height: 200
    //     x: parent.width - width - 20
    //     y: parent.height - height - 20
    //     z: 10
    //     MouseArea {
    //         anchors.fill: parent
    //         drag.target: parent
    //         drag.smoothed: true
    //     }
    // }


    NotificationPopup {

        id: notificationPopup
        property bool showPopup: false

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

        onConnected: client.doRequestAsync('GET', '/requested_pages', '')
                                      .then(function(msg) {
                                          requestedPages = msg['requested_pages']
                                          nav.construct()
                                          navBar.construct()
                                      })
    }

    // audio
    Connections {

        target: AudioBroadcaster

        function onReadyRead() {

            if(AudioBroadcaster.bytesAvailable < 2048 ||
                    !AudioBroadcaster.enableSend)
            {
                return
            }

            let data = AudioBroadcaster.readBase64(2048)

            let msg = {
                'type': 'speech',
                'data': data
            }

            client.sendTextMessage(JSON.stringify(msg))
        }
    }

    Settings {
        category: 'layout'
        property alias x: mainWindow.x
        property alias y: mainWindow.y
        property alias width: mainWindow.width
        property alias height: mainWindow.height
    }

}

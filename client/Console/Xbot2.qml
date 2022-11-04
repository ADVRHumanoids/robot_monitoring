import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

import "../sharedData.js" as SharedData

Page {

    id: root
    clip: true

    property var client: undefined

    SwipeView {

        id: swipe

        anchors.fill: parent

        anchors.margins: 10

        clip: true

    }

    RowLayout {

        id: layout

        anchors.fill: parent

        anchors.margins: 10

        spacing: 40

    }

    Plugins {

        id: plugins
        client: root.client

        Layout.fillHeight: true
    }

    Console {

        id: procConsole

        client: root.client

        Layout.fillHeight: true
        Layout.preferredWidth: parent.width / 2

    }

    footer: ToolBar {
        height: 20
        PageIndicator {
            anchors.centerIn: parent
            count: swipe.count
            currentIndex: swipe.currentIndex
        }
    }

    Component.onCompleted: {
        _handleResponsiveLayout()
        plugins.model = SharedData.pluginNames
        root.client.finalized.connect(function(){
            plugins.model = SharedData.pluginNames
        })
    }

    onWidthChanged: _handleResponsiveLayout()

    function _handleResponsiveLayout() {
        if(width < 576) {
            swipe.contentChildren = [procConsole, plugins]
            layout.children = []
            footer.visible = true
        }
        else {
            procConsole.parent = layout
            plugins.parent = layout
            swipe.contentChildren = []
            footer.visible = false
        }
    }


}

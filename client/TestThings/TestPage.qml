import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import xbot2_gui.common

import "../sharedData.js" as SharedData
import ".."

Item {

    id: root
    property ClientEndpoint client: undefined
    property bool mobileLayout: width < gridLeft.brSmall

    TabBar {
        id: bar
        width: parent.width
        TabButton {
            text: 'Process'
        }
        TabButton {
            text: 'Plugin'
        }
        y: mobileLayout ? 0 : -height
    }

    function responsiveLayout() {
        if(mobileLayout) {
            mainRow.children = []
            mainSwipe.contentChildren = [scrollLeft, scrollRight]
        }
        else {
            mainSwipe.contentChildren = []
            mainRow.children = [scrollLeft, scrollRight]
        }
    }

    onMobileLayoutChanged: {
        responsiveLayout()
    }

    Component.onCompleted: {

        // compute initial layout
        responsiveLayout()

        // register callbacks
        client.procMessageReceived.connect(onProcMessageReceived)
        client.pluginStatMessageReceived.connect(onPluginMessageReceived)

        // create process cards when available
        let process_list_received = function (msg) {
            SharedData.processInfo = msg
            procRepeater.model = msg
        }

        client.doRequest('GET', '/process/get_list', '', process_list_received)

        // create plugin cards when available
        let plugin_list_received = function (msg) {
            SharedData.pluginNames = msg.plugins
            pluginRepeater.model = msg.plugins
        }

        client.doRequest('GET', '/plugin/get_list', '', plugin_list_received)
    }

    Component.onDestruction: {
        client.procMessageReceived.disconnect(onProcMessageReceived)
        client.pluginStatMessageReceived.disconnect(onPluginMessageReceived)
    }

    SwipeView {
        id: mainSwipe
        width: parent.width
        anchors {
            top: bar.bottom
            bottom: parent.bottom
        }

        currentIndex: bar.currentIndex
    }

    RowLayout {

        id: mainRow
        width: parent.width
        anchors {
            top: bar.bottom
            bottom: parent.bottom
        }

        ScrollView {

            id: scrollLeft

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 1

            contentWidth: availableWidth
            contentHeight: gridLeft.height

            MaterialResponsiveGrid {

                id: gridLeft
                width: scrollLeft.contentWidth

                Label {
                    property int columnSpan: parent.columns
                    text: "Process control"
                    font.pixelSize: 28
                }

                Repeater {
                    id: procRepeater
                    model: 0

                    ProcessStatus {
                        processName: modelData.name
                        processState: modelData.status
                        processConfig: modelData.cmdline

                        onStart: processCmd(processName, 'start', processOptions)
                        onStop: processCmd(processName, 'stop', {})
                        onKill: processCmd(processName, 'kill', {})
                    }
                }

                Item {
                    width: 1
                    property int columnSpan: gridLeft.columns
                }

                RowLayout {
                    property int columnSpan: gridLeft.columns
                    spacing: 16

                    Label {
                        text: "Plugin control"
                        font.pixelSize: CommonProperties.font.h1
                    }

                    Item {
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                    }

                    Button {
                        text: 'Refresh'
                        onClicked: {
                            // create plugin cards when available
                            let plugin_list_received = function (msg) {
                                SharedData.pluginNames = msg.plugins
                                pluginRepeater.model = msg.plugins
                            }

                            client.doRequest('GET', '/plugin/get_list', '', plugin_list_received)
                        }
                    }

                }

                Repeater {
                    id: pluginRepeater
                    PluginStatus {
                        pluginName: modelData
                        onStart: pluginCmd(pluginName, 'start')
                        onStop: pluginCmd(pluginName, 'stop')
                        onAbort: pluginCmd(pluginName, 'abort')
                    }
                }
            }
        }

        ScrollView {

            id: scrollRight

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 1

            contentWidth: availableWidth
            contentHeight: availableHeight

            MaterialResponsiveGrid {

                id: gridRight
                width: scrollRight.contentWidth
                height: scrollRight.contentHeight
                sizeid: gridLeft.sizeid

                Label {
                    property int columnSpan: gridRight.columns
                    text: "Console output"
                    font.pixelSize: CommonProperties.font.h1
                }

                Console {
                    id: consoleItem
                    property int columnSpan: gridRight.columns
                    height: gridRight.contentHeight - y
                }
            }

        }
    }

    function onProcMessageReceived(msg) {
        // look for process with name msg.name
        for(let i = 0; i < procRepeater.count; i++) {

            var item_i = procRepeater.itemAt(i)

            // found!
            if(item_i.processName === msg.name) {

                if(msg.content === 'status')
                {
                    item_i.processState = msg.status
                }

                if(msg.content === 'output')
                {
                    let prefix = '[' + item_i.processName + '] '
                    if(msg.stdout.length > 0) {
                        consoleItem.consoleText.addText(prefix+msg.stdout)
                    }

                    if(msg.stderr.length > 0) {
                        consoleItem.consoleText.addText('<font color="red">' + prefix+msg.stderr + '</>')
                    }
                }

                break
            }
        }
    }

    function onPluginMessageReceived(msg) {
        for(let i = 0; i < pluginRepeater.count; i++) {
            let singlePlugin = pluginRepeater.itemAt(i)
            let pluginMsg = msg[singlePlugin.pluginName]
            singlePlugin.pluginPeriod = pluginMsg.expected_period
            singlePlugin.pluginCpuTime = pluginMsg.run_time
            singlePlugin.pluginState = pluginMsg.state
        }
    }


    function processCmd(name, cmd, opt) {

        let body = {
            name: name,
            cmd: cmd,
            options: opt
        }

        client.doRequest('PUT',
                         '/process/' + name + '/command/' + cmd,
                         JSON.stringify(body),
                         function(msg){console.log(JSON.stringify(msg))})
    }

    function pluginCmd(name, cmd) {

        client.doRequest('PUT',
                         '/plugin/' + name + '/command/' + cmd,
                         '',
                         function(msg){console.log(JSON.stringify(msg))})
    }

}

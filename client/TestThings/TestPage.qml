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
        responsiveLayout()
        client.procMessageReceived.connect(onProcMessageReceived)
        client.pluginStatMessageReceived.connect(onPluginMessageReceived)
        pluginRepeater.model = SharedData.pluginNames
        client.finalized.connect(function(){
            pluginRepeater.model = SharedData.pluginNames
        })
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

                Repeater{
                    id: procRepeater
                    model: SharedData.processInfo

                    ProcessStatus {
                        processName: modelData.name
                        processState: modelData.status
                        processConfig: modelData.cmdline

                        onStart: processCmd(processName, 'start', processOptions)
                        onStop: processCmd(processName, 'stop', {})
                        onKill: processCmd(processName, 'kill', {})

                    }

                }

                Label {
                    property int columnSpan: gridLeft.columns
                    text: "Plugin control"
                    font.pixelSize: CommonProperties.font.h1
                    topPadding: 10
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
        console.log(JSON.stringify(msg))
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
                         '/proc',
                         JSON.stringify(body),
                         function(msg){console.log(JSON.stringify(msg))})
    }

    function pluginCmd(name, cmd) {

        client.doRequest('PUT',
                         '/plugin/' + name + '/' + cmd,
                         '',
                         function(msg){console.log(JSON.stringify(msg))})
    }

}

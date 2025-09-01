import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common
import Main
import Font

import "Launcher.js" as Logic

MultiPaneResponsiveLayout {

    id: root

    property ClientEndpoint client

    property int numErrors: 0

    property var processMutedState: Object()


    LayoutClassHelper {
        id: layout
        targetWidth: root.width
    }

    ScrollView {

        property string iconText: 'Launcher'
        property string iconChar: MaterialSymbolNames.launcher

        id: leftScroll

        contentHeight: leftColumn.height
        contentWidth: availableWidth

        Column {

            id: leftColumn
            width: leftScroll.contentWidth
            spacing: 4

            // columns: root.layoutHelper.compact ? 1 : 2

            SectionHeader {

                visible: CommonProperties.config.showLauncherDashboard

                width: parent.width

                iconText: dashboard.visible ? '\uf077' : '\uf078'

                Button {
                    text: 'Refresh'
                    onClicked: {
                        dashboard.refresh()
                    }
                }

                text: 'Operation mode'

                onClicked: {
                    dashboard.visible = !dashboard.visible
                    // leftGrid.computeLayout()
                }

            }

            Dashboard {
                id: dashboard
                client: root.client
                width: parent.width
                visible: CommonProperties.config.showLauncherDashboard
                enabled: visible
            }

            Item {

                // spacer
                width: parent.width

                height: 16
                visible: dashboard.visible
            }

            SectionHeader {

                width: parent.width

                text: 'Process launcher'

                CheckBox {
                    id: showAllChk
                    text: 'Show All'
                    checked: false
                    onCheckedChanged: Qt.callLater(processLayout.computeLayout)
                }

                Button {
                    text: 'Refresh'
                    onClicked: Logic.requestProcessUpdate(processRepeater)
                }

                onClicked: {
                    processLayout.visible = !processLayout.visible
                }

                iconText: processLayout.visible ? '\uf077' : '\uf078'

            }

            MultiColumnLayout1 {

                id: processLayout

                width: parent.width
                columns: Math.ceil(width / 400.0)

                Repeater {

                    id: processRepeater

                    ProcessCard {

                        visible: (showAllChk.checked || modelData.visible) && processRepeater.visible
                        muted: !modelData.visible

                        processName: modelData.name
                        processState: modelData.status
                        processConfig: modelData.cmdline

                        objectName: `pcard_${processName}`

                        onStart: Logic.processCmd(processName, 'start', processOptions)
                        onStop: Logic.processCmd(processName, 'stop', {})
                        onKill: Logic.processCmd(processName, 'kill', {})

                        onMutedChanged: root.processMutedState[processName] = muted
                    }

                }

                CustoCommand {
                    id: customCmd
                    pageItem: root
                    onSubmitCommand: Logic.customCommand(machine, command, timeout)
                    // visible: processRepeater.visible
                }

            }

            Item {

                // spacer
                width: parent.width

                height: 16
                visible: processLayout.visible
            }

            SectionHeader {

                width: parent.width

                text: 'Plugin launcher'

                Button {
                    text: 'Refresh'
                    onClicked: {
                        Logic.requestPluginUpdate(pluginRepeater)
                    }
                }

                onClicked: {
                    pluginLayout.visible = !pluginLayout.visible
                    // leftGrid.computeLayout()
                }

                iconText: pluginLayout.visible ? '\uf077' : '\uf078'
            }

            MultiColumnLayout1 {

                id: pluginLayout

                width: parent.width
                columns: Math.ceil(width / 400.0)

                Repeater {

                    id: pluginRepeater

                    PluginCard {
                        visible: pluginRepeater.visible
                        pluginName: modelData
                        onStart: Logic.pluginCmd(pluginName, 'start')
                        onStop: Logic.pluginCmd(pluginName, 'stop')
                        onAbort: Logic.pluginCmd(pluginName, 'abort')
                    }

                }

            }
        }
    }

    // end left col


    LauncherConsoleItem {

        id: consoleItem

        property string iconText: 'Console'
        property string iconChar: MaterialSymbolNames.log

        objectName: 'Console'
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.preferredWidth: 1
        Layout.preferredHeight: 1

    }

    Component.onCompleted: Logic.construct(processRepeater,
                                           pluginRepeater)

    Connections {

        target: root.client

        function onProcessOutputReceived(msg) {
            Logic.onProcessOutputReceived(processRepeater, consoleItem, msg)
        }

        function onPluginStatMessageReceived(msg) {
            Logic.onPluginMessageReceived(pluginRepeater, msg)
        }

        function onObjectReceived(msg) {
            if(msg.type === 'proc_status') {
                Logic.onProcessStatusReceived(processRepeater, consoleItem, msg)
            }
        }
    }

}

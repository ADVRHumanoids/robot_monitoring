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



    ScrollView {

        property string iconText: 'Launcher'
        property string iconChar: MaterialSymbolNames.launcher

        id: leftScroll

        contentHeight: leftGrid.height
        contentWidth: availableWidth

        MultiColumnLayout {

            id: leftGrid
            width: leftScroll.contentWidth

            columns: root.layoutHelper.compact ? 1 : 2

            SectionHeader {

                objectName: 'sechdr'

                property int columnSpan: leftGrid.columns

                text: 'Process launcher'

                CheckBox {
                    id: showAllChk
                    text: 'Show All'
                    checked: false
                    onCheckedChanged: Qt.callLater(leftGrid.computeLayout)
                }

                Button {
                    text: 'Refresh'
                    onClicked: Logic.requestProcessUpdate(processRepeater)
                }
            }

            Repeater {

                id: processRepeater

                ProcessCard {

                    visible: showAllChk.checked || modelData.visible

                    processName: modelData.name
                    processState: modelData.status
                    processConfig: modelData.cmdline

                    objectName: `pcard_${processName}`

                    onStart: Logic.processCmd(processName, 'start', processOptions)
                    onStop: Logic.processCmd(processName, 'stop', {})
                    onKill: Logic.processCmd(processName, 'kill', {})
                }

            }

            CustoCommand {
                id: customCmd
                pageItem: root
                onSubmitCommand: Logic.customCommand(machine, command, timeout)
                visible: true
            }

            Item {

                // spacer
                property int columnSpan: leftGrid.columns

                height: 16
            }

            SectionHeader {

                property int columnSpan: leftGrid.columns

                text: 'Plugin launcher'

                Button {
                    text: 'Refresh'
                    onClicked: {
                        Logic.requestPluginUpdate(pluginRepeater)
                    }
                }
            }

            Repeater {

                id: pluginRepeater

                PluginCard {
                    pluginName: modelData
                    onStart: Logic.pluginCmd(pluginName, 'start')
                    onStop: Logic.pluginCmd(pluginName, 'stop')
                    onAbort: Logic.pluginCmd(pluginName, 'abort')
                }

            }
        }
    }



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

        function onProcMessageReceived(msg) {
            Logic.onProcMessageReceived(processRepeater, consoleItem, msg)
        }

        function onPluginStatMessageReceived(msg) {
            Logic.onPluginMessageReceived(pluginRepeater, msg)
        }

    }

}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common
import ".."
import "../Common"
import "Launcher.js" as Logic

MultiColumnPage {

    id: root

    property ClientEndpoint client

    columnItems: [leftItem, rightItem]
    mobileBreakpoint: leftGrid.brSmall

    property Item leftItem: ScrollView {

        id: leftScroll
        objectName: 'Plugin/Process'
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.preferredWidth: 1

        contentHeight: leftGrid.height
        contentWidth: availableWidth

        GridLayout {

            id: leftGrid
            width: leftScroll.contentWidth

            columns: 3

            SectionHeader {

                Layout.columnSpan: parent.columns

                text: 'Process launcher'

                Button {
                    text: 'Add'
                    enabled: false
                }

                Button {
                    text: 'Refresh'
                    onClicked: Logic.requestProcessUpdate(processRepeater)
                }
            }

            Repeater {

                id: processRepeater

                ProcessCard {
                    property var columnSpan: [4, 4, 4, 3]

                    processName: modelData.name
                    processState: modelData.status
                    processConfig: modelData.cmdline

                    onStart: Logic.processCmd(processName, 'start', processOptions)
                    onStop: Logic.processCmd(processName, 'stop', {})
                    onKill: Logic.processCmd(processName, 'kill', {})
                }

            }

            SectionHeader {

                Layout.columnSpan: parent.columns

                text: 'Plugin launcher'

                Button {
                    text: 'Refresh'
                    onClicked: Logic.requestPluginUpdate(pluginRepeater)
                }
            }

            Repeater {

                id: pluginRepeater

                PluginCard {
                    property var columnSpan: [4, 4, 4, 3]
                    pluginName: modelData
                    onStart: Logic.pluginCmd(pluginName, 'start')
                    onStop: Logic.pluginCmd(pluginName, 'stop')
                    onAbort: Logic.pluginCmd(pluginName, 'abort')
                }

            }
        }
    }



    property Item rightItem: LauncherConsoleItem {
        objectName: 'Console'
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.preferredWidth: 1
    }

    Component.onCompleted: Logic.construct(processRepeater,
                                           pluginRepeater,
                                           rightItem.mainConsole)

}

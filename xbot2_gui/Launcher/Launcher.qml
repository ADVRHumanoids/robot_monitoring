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

    property var categoryToProcessModel: Object()

    property var processStatusMap: Object()


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

                id: procSectionHeader

                property bool procVisible: true

                width: parent.width

                text: 'Process launcher'



                // CheckBox {
                //     id: showAllChk
                //     text: 'Show All'
                //     checked: false
                // }

                SmallToolButton {

                    id: collapseAllBtn

                    property bool collapseOnClick: true

                    text: MaterialSymbolNames.collapseAll
                    font.family: 'Material Symbols Outlined'
                    font.variableAxes: {'opsz': 48}
                    font.pixelSize: 16

                    visible: Object.keys(categoryToProcessModel).length > 1
                }

                SmallToolButton {
                    id: showAllChk
                    text: MaterialSymbolNames.visibility
                    font.family: 'Material Symbols Outlined'
                    font.variableAxes: {'opsz': 48}
                    font.pixelSize: 16
                    checkable: true
                }

                SmallToolButton {
                    text: MaterialSymbolNames.refresh
                    font.family: 'Material Symbols Outlined'
                    font.variableAxes: {'opsz': 48}
                    font.pixelSize: 16
                    onClicked: Logic.requestProcessUpdate()
                }

                onClicked: {
                    procVisible = !procVisible
                }

                iconText: procVisible ? '\uf077' : '\uf078'

            }

            Label {
                width: parent.width
                text: 'No process found'
                enabled: false
                font.pixelSize: CommonProperties.font.h2
                horizontalAlignment: Qt.AlignHCenter
                visible: processMainRepeater.count === 0 && procSectionHeader.procVisible
            }

            Repeater {

                id: processMainRepeater

                // model is the list of categories

                Column {

                    required property string modelData

                    visible: procSectionHeader.procVisible

                    width: parent.width

                    Connections {
                        target: showAllChk
                        function onCheckedChanged() {
                            Qt.callLater(processLayout.computeLayout)
                        }
                    }

                    Connections {
                        target: collapseAllBtn
                        function onClicked() {
                            if(collapseAllBtn.collapseOnClick) {
                                processLayout.visible = false
                            }
                            else {
                                processLayout.visible = true
                            }
                        }
                    }

                    SectionHeader {

                        pixelSize: CommonProperties.font.h4

                        visible: Object.keys(categoryToProcessModel).length > 1

                        width: parent.width

                        text: parent.modelData

                        onClicked: {
                            processLayout.visible = !processLayout.visible
                        }

                        iconText: processLayout.visible ? '\uf077' : '\uf078'

                        showBorderIfCompact: false

                    }


                    MultiColumnLayout1 {

                        id: processLayout

                        width: parent.width
                        columns: Math.ceil(width / 450.0)

                        Repeater {

                            id: processRepeater
                            model: root.categoryToProcessModel[modelData]

                            ProcessCard {

                                visible: (showAllChk.checked || modelData.visible) && procSectionHeader.procVisible
                                muted: !modelData.visible

                                processName: modelData.name
                                processState: processStatusMap[processName] ?? 'unknown'
                                processConfig: modelData.cmdline

                                objectName: `pcard_${processName}`

                                onStart: Logic.processCmd(processName, 'start', processOptions)
                                onStop: Logic.processCmd(processName, 'stop', {})
                                onKill: Logic.processCmd(processName, 'kill', {})

                                onMutedChanged: root.processMutedState[processName] = muted
                            }

                        }

                    }

                }

            }

            Item {

                // spacer
                width: parent.width

                height: 16
                visible: processMainRepeater.visible
            }

            SectionHeader {

                width: parent.width

                text: 'Plugin launcher'

                SmallToolButton {
                    text: MaterialSymbolNames.refresh
                    font.family: 'Material Symbols Outlined'
                    font.variableAxes: {'opsz': 48}
                    font.pixelSize: 16
                    onClicked: Logic.requestPluginUpdate(pluginRepeater)
                }

                onClicked: {
                    pluginStack.visible = !pluginStack.visible
                    // leftGrid.computeLayout()
                }

                iconText: pluginStack.visible ? '\uf077' : '\uf078'
            }

            StackLayout {

                id: pluginStack
                width: parent.width

                Control {

                    Layout.fillWidth: true

                    contentItem: ColumnLayout {

                        BusyIndicator {
                            running: !client.robotConnected
                            Layout.alignment: Qt.AlignHCenter
                        }

                        Label {
                            text: 'Robot not connected'
                            visible: !client.robotConnected
                            horizontalAlignment: Qt.AlignHCenter
                            Layout.alignment: Qt.AlignHCenter
                            font.pixelSize: CommonProperties.font.h4
                        }

                    }
                }

                MultiColumnLayout1 {

                    id: pluginLayout

                    enabled: client.robotConnected

                    Layout.fillWidth: true

                    columns: Math.ceil(width / 400.0)

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

    Component.onCompleted: Logic.construct()

    Connections {

        target: root.client

        function onProcessOutputReceived(msg) {
            Logic.onProcessOutputReceived(consoleItem, msg)
        }

        function onPluginStatMessageReceived(msg) {
            Logic.onPluginMessageReceived(pluginRepeater, msg)
        }

        function onObjectReceived(msg) {
            if(msg.type === 'proc_status') {
                Logic.onProcessStatusReceived(msg)
            }
        }

        function onRobotConnectedChanged() {
            if(client.robotConnected) {
                Logic.requestPluginUpdate(pluginRepeater, true)
            }
            else {
                pluginStack.currentIndex = 0
            }
        }
    }

}

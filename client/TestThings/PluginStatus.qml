import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

Rectangle {

    property bool pluginCanStart: pluginState === "Stopped" || pluginState === "Initialized"
    property string pluginName: 'PluginName'
    property string pluginState: 'Running'
    property real pluginCpuTime: -1.0
    property real pluginPeriod: 0.0

    signal start()
    signal stop()
    signal abort()

    id: root
    implicitHeight: mainCol.implicitHeight + 32
    color: pluginState === 'Running' ? CommonProperties.colors.ok :
                                       CommonProperties.colors.cardBackground
    radius: 4

    Column {

        id: mainCol
        width: parent.width - 32
        anchors.centerIn: parent
        spacing: 16

        Label {
            id: nameLabel
            text: root.pluginName
            font.bold: pluginState === 'Running'
            font.pixelSize: CommonProperties.font.h3
            color: pluginState === 'Running' ? CommonProperties.colors.secondaryText :
                                               CommonProperties.colors.primaryText
        }

        RowLayout {

            id: upperRow
            width: parent.width
            spacing: 16

            Button {
                id: startStopBtn
                text: pluginCanStart ? 'Start' : 'Stop'
                Layout.fillWidth: true
                onClicked: {
                    text === 'Start' ? start() : stop()
                }
            }

            Button {
                id: abortBtn
                text: 'Abort'
                Layout.fillWidth: true
                onClicked: {
                    abort()
                }
            }

        }

        RowLayout {

            width: parent.width
            spacing: 16

            Label {
                text: 'CPU time'
                Layout.alignment: Qt.AlignVCenter
            }

            ProgressBar {
                id: cpuBar
                from: 0
                to: pluginPeriod
                value: pluginCpuTime
                indeterminate: pluginCpuTime < 0
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignVCenter

                Label {
                    anchors.centerIn: parent
                    text: (pluginCpuTime*1000).toFixed(2) + ' ms'

                    Rectangle {
                        color: "white"
                        opacity: 0.2
                        radius: 4
                        anchors.centerIn: parent
                        width: parent.width + 6
                        height: parent.height + 6
                    }
                }
            }

        }

    }

}

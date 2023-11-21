import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import "../Common"

Item {

    // public

    property bool pluginCanStart: pluginState === "Stopped" || pluginState === "Initialized"
    property string pluginName: 'Homing'
    property string pluginState: 'Running'
    property real pluginCpuTime: -1.0
    property real pluginPeriod: 0.0
    readonly property bool pluginRunning: root.pluginState == 'Running'

    signal start()
    signal stop()
    signal abort()


    // private

    id: root

    implicitWidth: card.implicitWidth
    implicitHeight: card.implicitHeight

    height: card.height

    Card1 {

        id: card

        width: parent.width

        name: root.pluginName
        nameFont.bold: root.pluginRunning
        nameFont.pixelSize: CommonProperties.font.h3

        collapsed: true
        configurable: false

        backgroundColor: root.pluginRunning ?
                             CommonProperties.colors.ok :
                             defaultBackground

        toolButtons: [
            SmallToolButton {
                id: startQuickBtn
                text: root.pluginRunning ? '\uf04d' : '\uf04b'
                font.family: CommonProperties.fontAwesome.solid.family
                onClicked: root.pluginRunning ? root.stop() : root.start()
            }
        ]

        frontItem: ColumnLayout {
            anchors.fill: parent
            spacing: card.margins

            RowLayout {
                Layout.fillWidth: true
                spacing: card.margins
                Button {
                    Layout.fillWidth: true
                    text: root.pluginRunning ? 'Stop' : 'Start'
                    onClicked: root.pluginRunning ? root.stop() : root.start()
                }
                Button {
                    Layout.fillWidth: true
                    text: 'Abort'
                    onClicked: root.abort()
                }
            }

            LabeledProgressBar {
                id: cpuBar
                Layout.fillWidth: true
                from: 0
                to: pluginPeriod
                value: pluginCpuTime
                indeterminate: pluginCpuTime < 0
                Layout.alignment: Qt.AlignVCenter
                text: (pluginCpuTime*1000).toFixed(2) + ' ms'
            }
        }


    }
}

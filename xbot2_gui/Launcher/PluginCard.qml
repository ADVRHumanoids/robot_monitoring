import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Item {

    // public

    property string pluginName: 'Homing'
    property string pluginState: ''
    property real pluginCpuTime: -1.0
    property real pluginPeriod: 0.0
    readonly property bool pluginRunning: root.pluginState == 'Running'
    readonly property bool pluginStarting: root.pluginState == 'Starting'
    readonly property bool pluginStopping: root.pluginState == 'Stopping'
    readonly property bool pluginStoppable: pluginRunning || pluginStarting

    signal start()
    signal stop()
    signal abort()


    // private

    id: root

    enabled: pluginState !== '' && pluginState !== 'InitFailed'

    property var colorMap: {
        'Aborted': CommonProperties.colors.err,
        'Starting': Qt.lighter(CommonProperties.colors.ok, 3),
        'Stopping': CommonProperties.colors.err,
        'Running': Qt.lighter(CommonProperties.colors.ok),
        'Initialized': card.defaultBackground,
        'Stopped': card.defaultBackground,
        '': card.defaultBackground
    }

    implicitWidth: card.implicitWidth
    implicitHeight: card.implicitHeight

    Card1 {

        id: card

        width: parent.width
        height: parent.height
        verticalMargins: -6

        name: root.pluginName
        nameFont.bold: root.pluginRunning
        nameFont.pixelSize: CommonProperties.font.h3

        collapsed: true
        configurable: false

        // color management
        backgroundColor: colorMap[pluginState]
        borderWidth: 2
        borderColor: pluginState === 'Aborted' ? Qt.darker(backgroundColor) : Qt.lighter(backgroundColor)

        SequentialAnimation on backgroundColor {

            loops: Animation.Infinite

            running: root.pluginState === 'Starting' ||
                     root.pluginState === 'Stopping'

            alwaysRunToEnd: false

            ColorAnimation {
                from: colorMap[pluginState]
                to: card.defaultBackground
                duration: 555
                easing {
                    type: Easing.OutSine
                }
            }

            ColorAnimation {
                from: card.defaultBackground
                to: colorMap[pluginState]
                duration: 555
                easing {
                    type: Easing.InSine
                }
            }

            onFinished: card.backgroundColor = Qt.binding(() => colorMap[pluginState])

        }

        toolButtons: [
            SmallToolButton {
                id: startQuickBtn
                text: root.pluginStoppable ? '\uf04d' : '\uf04b'
                font.family: CommonProperties.fontAwesome.solid.family
                onClicked: root.pluginStoppable ? root.stop() : root.start()
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
                    text: root.pluginStoppable ? 'Stop' : 'Start'
                    onClicked: root.pluginStoppable ? root.stop() : root.start()
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

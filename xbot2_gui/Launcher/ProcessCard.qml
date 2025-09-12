import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Font

Item {

    // public

    property bool processRunning: processState === 'Running' || processState === 'Waiting'
    property bool processKilled: processState === 'Killed'
    property string processState: 'Stopped'
    property string processName: 'ProcessName'
    property alias processConfig: configPanel.description
    property alias processOptions: configPanel.options
    property alias muted: muteSwitch.checked

    signal start()
    signal stop()
    signal kill()

    implicitHeight: card.implicitHeight

    // private
    property var colorMap: {
        'Running':  Qt.lighter(CommonProperties.colors.ok),
        'Stopped': card.defaultBackground,
        'Killed': CommonProperties.colors.err,
        'Waiting': Qt.lighter(CommonProperties.colors.ok, 3),
        'Killing': CommonProperties.colors.err
    }

    id: root
    Layout.maximumHeight: card.Layout.maximumHeight

    Card1 {

        id: card

        width: root.width
        height: root.height
        verticalMargins: -6

        name: root.processName
        nameFont.pixelSize: CommonProperties.font.h3

        collapsed: true

        // status icon (mute)
        statusIcon.text: MaterialSymbolNames.volumeOff
        statusIcon.font.family: 'Material Symbols Outlined'
        statusIcon.font.pixelSize: 18
        statusIcon.visible: muteSwitch.checked

        // color management
        backgroundColor: colorMap[processState]
        borderWidth: 2
        borderColor: processKilled ? Qt.darker(backgroundColor) : Qt.lighter(backgroundColor)



        SequentialAnimation on backgroundColor {

            loops: Animation.Infinite

            running: root.processState === 'Waiting' ||
                     root.processState === 'Killing'

            alwaysRunToEnd: true

            ColorAnimation {
                from: colorMap[processState]
                to: card.defaultBackground
                duration: 555
                easing {
                    type: Easing.OutSine
                }
            }

            ColorAnimation {
                from: card.defaultBackground
                to: colorMap[processState]
                duration: 555
                easing {
                    type: Easing.InSine
                }
            }

            onFinished: card.backgroundColor = Qt.binding(() => colorMap[processState])

        }

        toolButtons: [

            SmallToolButton {
                id: startQuickBtn
                text: root.processRunning ? '\uf04d' : '\uf04b'
                font.family: CommonProperties.fontAwesome.solid.family
                onClicked: root.processRunning ? root.stop() : root.start()
            }

        ]

        frontItem: GridLayout {
            // anchors.fill: parent
            columns: 2
            height: implicitHeight
            anchors.fill: parent
            Button {
                text: root.processRunning ? 'Stop' : 'Start'
                onClicked: root.processRunning ? root.stop() : root.start()
                Layout.fillWidth: true
            }
            Button {
                text: 'Kill'
                onClicked: root.kill()
                Layout.fillWidth: true
            }
            Switch {
                Layout.columnSpan: 2

                id: muteSwitch
                text: 'Mute'
                checked: false
            }
        }

        backItem: ConfigurePanel {
            id: configPanel
            enabled: !root.processRunning
            anchors.fill: parent
        }

        onApplyConfiguration: {
            configPanel.applySettings()
        }
    }
}

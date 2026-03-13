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
    property var processInfo: Object()
    property alias muted: muteSwitch.checked
    property bool deletable: processInfo.is_custom || false

    signal start()
    signal stop()
    signal kill()
    signal deleteRequest()
    signal procEditRequest()

    implicitHeight: card.implicitHeight

    // private
    id: root

    property var colorMap: {
        'Running':  Qt.lighter(CommonProperties.colors.ok),
        'Stopped': card.defaultBackground,
        'Killed': CommonProperties.colors.err,
        'Waiting': Qt.lighter(CommonProperties.colors.ok, 3),
        'Killing': CommonProperties.colors.err,
        'unknown': 'lightgray'
    }


    // impl

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
        // backgroundColor: colorMap[processState]
        borderWidth: 2
        borderColor: processKilled ? Qt.darker(badge.color, 1.1) : Qt.lighter(badge.color, 1.1)



        toolButtons: [

            SmallToolButton {
                id: startQuickBtn
                text: root.processRunning ? '\uf04d' : '\uf04b'
                font.family: CommonProperties.fontAwesome.solid.family
                onClicked: root.processRunning ? root.stop() : root.start()
            }
        ]

        badgeItem: Rectangle {
            id: badge
            implicitHeight: 8
            implicitWidth: 8
            radius: 4
            color: colorMap[processState]

            SequentialAnimation on color {

                loops: Animation.Infinite

                running: root.processState === 'Waiting' ||
                         root.processState === 'Killing' ||
                         root.processState === 'unknown'

                alwaysRunToEnd: true

                ColorAnimation {
                    from: colorMap[processState]
                    to: badge.color
                    duration: 555
                    easing {
                        type: Easing.OutSine
                    }
                }

                ColorAnimation {
                    from: badge.color
                    to: colorMap[processState]
                    duration: 555
                    easing {
                        type: Easing.InSine
                    }
                }

                onFinished: badge.color = Qt.binding(() => colorMap[processState])

            }

        }

        frontItem: GridLayout {
            // anchors.fill: parent
            columns: 2
            height: implicitHeight
            anchors.fill: parent

            Button {
                text: root.processRunning ? 'Stop' : 'Start'
                onClicked: root.processRunning ? root.stop() : root.start()
                Layout.fillWidth: true
                highlighted: true
            }

            Button {
                text: 'Kill'
                onClicked: root.kill()
                Layout.fillWidth: true
            }

            FramedControl {
                text: 'Mute'
                subtext: 'Disable console output'
                Layout.columnSpan: 2
                Layout.fillWidth: true
                Switch {
                    id: muteSwitch
                    checked: false
                }
            }

            FramedControl {
                Layout.columnSpan: 2
                Layout.fillWidth: true
                text: 'Delete process'
                subtext: root.processRunning ? 'Process cannot be removed while running' :
                                               'Remove this process from the GUI server (custom process only)'
                visible: root.deletable
                enabled: !root.processRunning
                Button {
                    text: 'Delete'
                    onClicked: root.deleteRequest()
                }
            }
        }

        backItem: ConfigurePanel {
            id: configPanel
            enabled: !root.processRunning
            anchors.fill: parent
            info: root.processInfo

            onProcEditRequested: root.procEditRequest()
        }

        onApplyConfiguration: {
            configPanel.applySettings()
        }
    }
}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import "../Common"

Item {

    // public

    property bool processRunning: processState === 'Running'
    property bool processKilled: processState === 'Killed'
    property string processState: 'Stopped'
    property string processName: 'ProcessName'
    property alias processConfig: configPanel.description
    property alias processOptions: configPanel.options

    signal start()
    signal stop()
    signal kill()

    height: card.height


    // private

    id: root
    Layout.maximumHeight: card.Layout.maximumHeight

    Card1 {

        id: card

        width: root.width

        name: root.processName
        nameFont.pixelSize: CommonProperties.font.h3

        collapsed: true

        backgroundColor: root.processRunning ?
                             CommonProperties.colors.ok :
                             (root.processKilled ? CommonProperties.colors.err : defaultBackground)

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
            Button {
                text: root.processRunning ? 'Stop' : 'Start'
                onClicked: root.processRunning ? root.stop() : root.start()
            }
            Button {
                text: 'Kill'
                onClicked: root.kill()
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

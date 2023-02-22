import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import xbot2_gui.common
import "../Common"

Item {

    // public

    property bool processRunning: processState === 'Running'
    property string processState: 'Stopped'
    property string processName: 'ProcessName'
    property alias processConfig: configPanel.description
    property alias processOptions: configPanel.options

    signal start()
    signal stop()
    signal kill()


    // private

    id: root
    implicitWidth: card.implicitWidth
    implicitHeight: card.implicitHeight
    height: card.height

    Card {

        id: card

        width: root.width

        name: root.processName
        nameFont.pixelSize: CommonProperties.font.h3

        hidden: true

        backgroundColor: root.processRunning ?
                             CommonProperties.colors.ok :
                             defaultBackground

        toolButtons: [
            SmallToolButton {
                id: startQuickBtn
                text: root.processRunning ? '\uf04d' : '\uf04b'
                font.family: CommonProperties.fontAwesome.solid.family
                onClicked: root.processRunning ? root.stop() : root.start()
            }
        ]

        frontItem: GridLayout {
            anchors.fill: parent
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
            height: 400
            id: configPanel
            enabled: !root.processRunning
            anchors.fill: parent
        }

        onApplyConfiguration: {
            configPanel.applySettings()
        }
    }
}

import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

import "../Console"

Rectangle {

    id: root
    property bool processRunning: true
    property string processName: 'Xbot2'
    implicitHeight: mainCol.implicitHeight + 32
    color: Material.background
    radius: 4

    Column {

        id: mainCol
        width: parent.width - 32
        anchors.centerIn: parent
        spacing: 16

        RowLayout {

            id: upperRow
            width: parent.width
            spacing: 16


            Label {
                id: nameLabel
                text: root.processName
                font.bold: root.processRunning
                font.pixelSize: 16
                color: root.processRunning ? Material.color(Material.Green) :
                                             Material.primaryTextColor
            }


            Button {
                id: startStopBtn
                text: 'Start'
                Layout.fillWidth: true
            }

            Button {
                id: abortBtn
                text: 'Abort'
                Layout.fillWidth: true
            }

        }

        RowLayout {

            width: parent.width

            Switch {
                text: 'Mute'
                Layout.fillWidth: true
            }

            Button {
                text: configPanel.height == 0 ? 'Configure' : 'Cancel'
                Layout.fillWidth: true
                onReleased: {

                    if(text === 'Cancel') {
                        configPanel.height = 0
                        return
                    }

                    configPanel.description = {
                        'Verbose': {
                            'type': 'check',
                            'default': true
                        },
                        'HW type': {
                            'type': 'combo',
                            'options': ['ciao', 'gatto', 'miao']
                        },
                        'Name': {
                            'type': 'text',
                            'help': 'Write something original'
                        }
                    }

                    configPanel.height = 300
                }

            }

        }

        ConfigurePanel {
            id: configPanel
            width: parent.width
            height: 0
            clip: true
            x: 0

            Behavior on height {
                NumberAnimation {
                    duration: 500
                    easing.type: Easing.OutQuad
                }
            }

            onCloseRequested: {
                height = 0
            }
        }

    }

}

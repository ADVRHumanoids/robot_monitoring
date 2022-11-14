import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

import "../Console"

Rectangle {

    property bool processRunning: processState === 'Running'
    property string processState: 'Running'
    property string processName: ''
    property alias processConfig: configPanel.description
    property alias processOptions: configPanel.options

    signal start()
    signal stop()
    signal kill()


    id: root
    implicitHeight: mainCol.implicitHeight + 32
    color: processRunning ? CommonProperties.color.ok :
                            CommonProperties.color.cardBackground
    radius: 4

    Column {

        id: mainCol
        width: parent.width - 32
        anchors.centerIn: parent
        spacing: 16

        Label {
            id: nameLabel
            text: root.processName
            font.bold: root.processRunning
            font.pixelSize: CommonProperties.font.h3
            color: root.processRunning ? CommonProperties.color.secondaryText :
                                         CommonProperties.color.primaryText
        }

        RowLayout {

            id: upperRow
            width: parent.width
            spacing: 16


            Button {
                id: startStopBtn
                text: processRunning ? 'Stop' : 'Start'
                Layout.fillWidth: true
                onClicked: {
                    if(text === 'Start') start()
                    if(text === 'Stop') stop()
                }
            }

            Button {
                id: abortBtn
                text: 'Abort'
                Layout.fillWidth: true
                onClicked: {
                    kill()
                }
            }

        }

        RowLayout {

            width: parent.width

            Switch {
                text: 'Mute'
                Layout.fillWidth: true
            }

            Button {
                enabled: !processRunning
                text: configPanel.height == 0 ? 'Configure' : 'Cancel'
                Layout.fillWidth: true
                onReleased: {

                    if(text === 'Cancel') {
                        configPanel.height = 0
                        return
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

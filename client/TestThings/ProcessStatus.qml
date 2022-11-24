import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

import "../Console"

Flipable {

    property bool processRunning: processState === 'Running'
    property string processState: 'Running'
    property string processName: ''
    property alias processConfig: configPanel.description
    property alias processOptions: configPanel.options

    signal start()
    signal stop()
    signal kill()

    id: root
    height: flipped ? backSide.height : frontSide.height
    property bool flipped: false
    Behavior on height {
        NumberAnimation {
            duration: 500
            easing.type: Easing.OutQuad
        }
    }

    back: Rectangle {
        id: backSide
        width: root.width
        height: 300
        color: frontSide.color
        radius: 4

        ConfigurePanel {
            enabled: !processRunning
            id: configPanel
            anchors.fill: parent
            anchors.margins: 16
            onCloseRequested: {
                root.flipped = false
            }
        }
    }

    front: Rectangle {

        id: frontSide

        color: processRunning ? CommonProperties.colors.ok :
                                CommonProperties.colors.cardBackground
        radius: 4

        width: root.width
        implicitHeight: mainCol.implicitHeight + 32

        Column {

            id: mainCol
            width: parent.width - 32
            anchors.centerIn: parent
            spacing: 16

            RowLayout {

                width: parent.width
                spacing: 16

                Label {
                    id: nameLabel
                    text: root.processName
                    font.bold: root.processRunning
                    font.pixelSize: CommonProperties.font.h3
                    color: root.processRunning ? CommonProperties.colors.secondaryText :
                                                 CommonProperties.colors.primaryText
                }

                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }

                Label {
                    text: '\u2699'
                    font.pixelSize: CommonProperties.font.h2
                    MouseArea {
                        anchors.fill: parent
                        onClicked: root.flipped = !root.flipped
                    }
                }

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

        }

    }

    transform: Rotation {
        id: rotation
        origin.x: root.width/2
        origin.y: root.height/2
        axis.x: 0; axis.y: 1; axis.z: 0     // set axis.y to 1 to rotate around y-axis
        angle: 0    // the default angle
    }

    states: State {
        name: "back"
        PropertyChanges { target: rotation; angle: 180 }
        when: root.flipped
    }

    transitions: Transition {
        NumberAnimation {
            target: rotation
            property: "angle"
            duration: 500
            easing.type: Easing.OutQuad
        }
    }



}

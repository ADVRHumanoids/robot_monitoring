import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

import Common
import Main

import "Dashboard.js" as Logic


Control {

    //
    id: root

    property ClientEndpoint client

    property var stateMap

    property list<string> stateNames

    property string activeState

    property Button readyBtn

    property TextArea log: TextArea {}

    property bool startStopPending: false

    function refresh() {
        Logic.refresh()
    }

    padding: 4

    contentItem: GridLayout {

        // anchors.fill: parent

        columns: width > 450 ? 2 : 1

        uniformCellWidths: true

        columnSpacing: 16

        Repeater {

            model: root.stateNames

            Button {

                id: btn

                property bool stateActive: root.activeState === modelData
                property var backgroundColor
                property bool activationInProgress: false

                Layout.fillHeight: true
                Layout.fillWidth: true

                Rectangle {
                    color: 'transparent'
                    border.color: 'lightgreen'
                    border.width: 4
                    width: parent.background.width + 4
                    height: parent.background.height + 4
                    radius: height/2
                    anchors.centerIn: parent
                    visible: btn.stateActive
                }

                text: root.stateMap[modelData].nice_name || Logic.toTitleCase(modelData)

                font.pixelSize: CommonProperties.font.h3

                enabled: root.activeState !== "inactive"

                onClicked: {

                    if(stateActive) {
                        return
                    }

                    Logic.changeState(btn, modelData)

                }

                Component.onCompleted: {
                    backgroundColor = background.color
                    if(modelData === 'ready') readyBtn = btn
                }

                SequentialAnimation on Material.background {

                    loops: Animation.Infinite

                    running: btn.activationInProgress

                    alwaysRunToEnd: true

                    ColorAnimation {
                        from: backgroundColor
                        to: 'lightgreen'
                        duration: 400
                    }

                    ColorAnimation {
                        from: 'lightgreen'
                        to: backgroundColor
                        duration: 400
                    }

                }


            }
        }

        // MenuSeparator {
        //     Layout.alignment: Qt.AlignHCenter
        // }

        Button {

            id: startStopBtn
            property color backgroundColor
            text: root.activeState === 'inactive' ? 'Start' : 'Stop'

            Layout.fillHeight: true
            Layout.fillWidth: true
            font.pixelSize: CommonProperties.font.h3

            Component.onCompleted: backgroundColor = background.color

            Rectangle {
                id: startStopBtnBorder
                color: 'transparent'
                border.color: root.activeState === 'inactive' ?
                                  'lightgreen' :
                                  CommonProperties.colors.err
                border.width: 4
                width: parent.background.width + 4
                height: parent.background.height + 4
                radius: height/2
                anchors.centerIn: parent
            }

            SequentialAnimation on Material.background {

                loops: Animation.Infinite

                running: root.startStopPending

                alwaysRunToEnd: true

                ColorAnimation {
                    from: startStopBtn.backgroundColor
                    to: startStopBtnBorder.border.color
                    duration: 400
                }

                ColorAnimation {
                    from: startStopBtnBorder.border.color
                    to: startStopBtn.backgroundColor
                    duration: 400
                }

                onFinished: startStopBtn.Material.background = startStopBtn.backgroundColor

            }

            onClicked: stopPopup.open()

            Popup {

                id: stopPopup
                anchors.centerIn: Overlay.overlay
                modal: true
                focus: true
                padding: 16

                ColumnLayout {

                    spacing: 16

                    Text {

                        text: startStopBtn.text === 'Start' ?
                                  'Release the <b>emergency button</b> to power up the motors, then confirm.' :
                                  'Press the <b>emergency button</b> to engage brakes, then confirm.'

                        font.pixelSize: CommonProperties.font.h3

                        color: palette.active.text
                    }

                    Button {

                        text: 'Confirm'

                        font.pixelSize: CommonProperties.font.h3

                        Layout.alignment: Qt.AlignHCenter

                        onClicked: {

                            if(startStopBtn.text === 'Start') {
                                Logic.startRobot()
                            }
                            else {
                                Logic.stopRobot()
                            }

                            stopPopup.close()
                        }
                    }
                }
            }
        }
    }


    Component.onCompleted: {

        Logic.refresh()

    }

    Connections {

        target: client

        function onObjectReceived(msg) {

            if(msg.type !== 'dashboard_msg') {
                return
            }

            root.activeState = msg.active_state || root.activeState

            if(msg.msg)
            {
                log.append('[status] ' + msg.msg)
            }

        }
    }

}


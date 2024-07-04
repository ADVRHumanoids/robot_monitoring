import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

import Common
import Main

import "Dashboard.js" as Logic


MultiPaneResponsiveLayout {

    //
    id: root

    property ClientEndpoint client

    property var stateMap

    property list<string> stateNames

    property string activeState

    ColumnLayout {

        anchors.fill: parent

        Repeater {

            model: root.stateNames

            Button {

                id: btn

                property bool stateActive: root.activeState === modelData
                property var backgroundColor
                property bool activationInProgress: false

                Layout.fillHeight: true
                Layout.fillWidth: true

                text: root.stateMap[modelData].nice_name || Logic.toTitleCase(modelData)
                font.pixelSize: CommonProperties.font.h3

                enabled: root.activeState !== "inactive"

                onClicked: {

                    if(stateActive) {
                        return
                    }

                    Logic.changeState(btn, modelData)

                }

                Component.onCompleted: backgroundColor = palette.active.button

                Material.background: (stateActive) ?
                                      CommonProperties.colors.ok : backgroundColor

                SequentialAnimation on Material.background {

                    loops: Animation.Infinite

                    running: btn.activationInProgress

                    alwaysRunToEnd: true

                    ColorAnimation {
                        from: 'lightgreen'
                        to: backgroundColor
                        duration: 400
                    }

                    ColorAnimation {
                        from: backgroundColor
                        to: 'lightgreen'
                        duration: 400
                    }

                    onFinished: {
                        console.log('finished')
                        btn.Material.background = Qt.binding(
                                    function(){
                                        return (stateActive) ? CommonProperties.colors.ok : backgroundColor
                                    })
                    }

                }


            }
        }

        MenuSeparator {
            Layout.alignment: Qt.AlignHCenter
        }

        Button {

            text: root.activeState === 'inactive' ? 'Start' : 'Stop'

            Layout.fillHeight: true
            Layout.fillWidth: true
            font.pixelSize: CommonProperties.font.h3

            Material.background: root.activeState === 'inactive' ?
                                     CommonProperties.colors.ok :
                                     CommonProperties.colors.err

            onClicked: {
                if(text === 'Stop') {
                    Logic.triggerSafety()
                    stopPopup.open()
                }
                else {
                    Logic.startRobot()
                }


            }

            Popup {
                id: stopPopup
                anchors.centerIn: Overlay.overlay
                modal: true
                focus: true
                ColumnLayout {
                    spacing: 16
                    Text {
                        text: 'Press the <b>emergency button</b> to engage brakes, then confirm.'
                        font.pixelSize: CommonProperties.font.h3
                        color: palette.active.text
                    }
                    Button {
                        text: 'Confirm'
                        font.pixelSize: CommonProperties.font.h3
                        Layout.alignment: Qt.AlignHCenter

                        onClicked: {
                            Logic.stopRobot()
                            stopPopup.close()
                        }
                    }
                }
            }
        }
    }

    ColumnLayout {

        TextArea {
            id: log
            placeholderText: 'Log'
            Layout.fillHeight: true
            Layout.fillWidth: true
        }

    }

    Component.onCompleted: {

        client.doRequestAsync('GET', '/dashboard/get_states', '')
            .then(function(res) {

                root.stateMap = res

                let tmp_stateNames = []

                for (const [key, value] of Object.entries(res)) {
                    tmp_stateNames.push(key)
                }

                root.stateNames = tmp_stateNames

                console.log(`${root.stateNames}`)

            })

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


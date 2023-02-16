import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "../Cartesian"
import ".."
import "../Cartesian/cartesian.js" as Logic

Item {

    id: root

    property ClientEndpoint client: undefined
    property var vref: [0, 0, 0, 0, 0, 0]
    property alias taskCombo: taskCombo
    property bool portrait: width < height

    property real maxLinearV: maxSpeedLinearSpinBox.value
    property real maxAngularV: maxSpeedLinearSpinBox.value


    Component.onCompleted: {
        Logic.updateTaskNames()
    }

    TabBar {
        id: bar
        anchors.top: parent.top
        width: parent.width
        TabButton {
            text: 'Joypad'
        }
        TabButton {
            text: 'Settings'
        }
    }

    SwipeView {

        id: swipe
        currentIndex: bar.currentIndex
        interactive: false

        anchors.top: bar.bottom
        anchors.bottom: parent.bottom
        width: parent.width

        Item {

            id: joypad

            GridLayout {

                anchors.fill: parent
                anchors.margins: CommonProperties.geom.spacing
                columnSpacing: CommonProperties.geom.spacing
                rowSpacing: CommonProperties.geom.spacing

                rows: root.portrait ? 2 : 1
                columns: root.portrait ? 1 : 2

                Repeater {

                    model: 2

                    VideoStream {
                        Layout.fillHeight: true
                        Layout.fillWidth: true

                        property string streamName
                    }

                }

            }


            Pad  {
                anchors {
                    left: parent.left
                    bottom: parent.bottom
                    margins: 48
                }

                side: Math.min(200, parent.width/2 - 64)

                opacity: 0.7

                onJoystickMoved: {
                    vref[0] = joyY*maxLinearV
                    vref[1] = -joyX*maxLinearV
                    Logic.sendVref()
                }

            }

            Pad  {
                anchors {
                    right: parent.right
                    bottom: parent.bottom
                    margins: 48
                }

                horizontalOnly: true

                side: Math.min(200, parent.width/2 - 64)

                opacity: 0.7

                onJoystickMoved: {
                    vref[5] = -joyX*maxAngularV
                    Logic.sendVref()
                }
            }
        }

        Item {

            id: settings

            MaterialResponsiveGrid {

                anchors.fill: parent

                Card {
                    name: 'Camera'
                    frontItem: GridLayout {

                        anchors.fill: parent

                            Repeater {

                            Label { text: 'Camera 1' }

                            ComboBox {

                            }

                            TextField {

                            }

                        }
                    }
                }
            }

            GridLayout {

                anchors.fill: parent
                anchors.margins: 16

                columns: 4

                Label {
                    text: 'Tasks'
                }

                ComboBox {
                    id: taskCombo
                    model: []
                    Layout.fillWidth: true
                    onCurrentTextChanged: {
                        // get state from server to update enable/disable btn
                        Logic.taskIsEnabled(currentText,
                                            function (is_active) {
                                                enableDisableBtn.taskActive = is_active
                                            })
                    }
                }

                Button {
                    id: enableDisableBtn
                    text: taskActive ? 'Disable' : 'Enable'
                    property bool taskActive: true
                    onReleased: {
                        let callback = function () {
                            Logic.taskIsEnabled(taskCombo.currentText,
                                                (is_active) => {
                                                    taskActive = is_active
                                                })
                        }
                        if(taskActive) {
                            Logic.disableTask(taskCombo.currentText, callback)
                        }
                        else {
                            Logic.enableTask(taskCombo.currentText, callback)
                        }
                    }
                }

                Button {
                    text: 'Refresh'
                    onReleased: {
                        Logic.updateTaskNames()
                    }
                }

                Label {
                    text: 'Max speed (linear)'
                }

                DoubleSpinBox {
                    Layout.columnSpan: 2
                    id: maxSpeedLinearSpinBox
                    from: 0.0
                    to: 2.0
                }
            }
        }
    }
}

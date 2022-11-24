import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "../Cartesian"
import "../Video"
import ".."
import "../Cartesian/cartesian.js" as Logic

Item {

    id: root

    property ClientEndpoint client: undefined
    property var vref: [0, 0, 0, 0, 0, 0]
    property alias taskCombo: taskCombo
    property bool portrait: width < height

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

            VideoStreamFrontend {
                anchors.top: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.margins: 16
                height: (parent.width - 32)/1.33 > parent.height - 32 ?
                            parent.height - 32 : (parent.width - 32)/1.33
                width: (parent.height - 32)*1.33 > parent.width - 32 ?
                           parent.width - 32 : (parent.height - 32)*1.33

                client: root.client
                property var column: [0, 0, 2, 2]
                property var columnSpan: [4, 8, 8, 8]

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
                    vref[0] = joyY
                    vref[1] = -joyX
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
                    vref[5] = -joyX
                    Logic.sendVref()
                }
            }
        }

        Item {

            id: settings

            GridLayout {

                anchors.fill: parent
                anchors.margins: 16

                columns: 3

                Label {
                    text: 'Tasks'
                }

                ComboBox {
                    id: taskCombo
                    model: []
                    Layout.fillWidth: true
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

                SpinBox {
                    id: spinbox
                    Layout.columnSpan: 2
                    from: 0
                    value: 10
                    to: 100
                    stepSize: 10
                    property int decimals: 1
                    property real realValue: value / 100

                    validator: DoubleValidator {
                        bottom: Math.min(spinbox.from, spinbox.to)
                        top:  Math.max(spinbox.from, spinbox.to)
                    }

                    textFromValue: function(value, locale) {
                        return Number(value / 100).toLocaleString(locale, 'f', spinbox.decimals)
                    }

                    valueFromText: function(text, locale) {
                        return Number.fromLocaleString(locale, text) * 100
                    }
                }
            }
        }
    }
}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Main
import Common

Item {

    property ClientEndpoint client

    Item {
        id: sliderModel
        SliderModelItem {
            name: 'X'
            max: maxStiffTransSpin.value
        }
        SliderModelItem {
            name: 'Y'
            max: maxStiffTransSpin.value
        }
        SliderModelItem {
            name: 'Z'
            max: maxStiffTransSpin.value
        }
        SliderModelItem {
            name: 'r'
            max: maxStiffRotSpin.value
        }
        SliderModelItem {
            name: 'p'
            max: maxStiffRotSpin.value
        }
        SliderModelItem {
            name: 'y'
            max: maxStiffRotSpin.value
        }
    }

    Frame {

        anchors.verticalCenter: parent.verticalCenter
        anchors.right: parent.right
        anchors.margins: 16

        ColumnLayout {

            RowLayout {

                Label {
                    id: maxStiffTransLabel
                    text: 'Max Stiffness (trans.)  '
                }

                SpinBox {
                    id: maxStiffTransSpin
                    editable: true
                    from: 0
                    to: 20000
                    stepSize: value < 1000 ? 100 : 1000
                    value: 10000
                }

            }

            RowLayout {

                Label {
                    text: 'Max Stiffness (rot.)    '
                    Layout.preferredWidth: maxStiffTransLabel.width
                }

                SpinBox {
                    id: maxStiffRotSpin
                    editable: true
                    from: 0
                    to: 20000
                    stepSize: value < 1000 ? 100 : 1000
                    value: 1000
                }

            }

            Repeater {

                model: sliderModel.children

                RowLayout {

                    Label {
                        text: modelData.name
                    }

                    Slider {
                        id: slider
                        Layout.fillWidth: true
                        to: modelData.max
                    }

                    Label {
                        text: slider.value.toFixed(0)
                        Layout.minimumWidth: 50
                        topPadding: 4
                        bottomPadding: 4
                        rightPadding: 4
                        leftPadding: 4
                        background: Rectangle {
                            color: 'transparent'
                            border.color: Qt.rgba(1, 1, 1, 0.3)
                            border.width: 1
                            radius: 4
                        }
                    }

                }

            }

            RowLayout {
                spacing: 16
                Button {
                    text: 'Reset'
                    Layout.fillWidth: true
                }
                Button {
                    text: 'Apply'
                    Layout.fillWidth: true
                }
            }

            RowLayout {
                spacing: 16
                Switch {
                    id: gcompSwitch
                    text: 'Gcomp'
                    Layout.fillWidth: true
                }
                Switch {
                    text: 'Ros Ctrl'
                    enabled: !gcompSwitch.checked
                    Layout.fillWidth: true
                }
            }

        }
    }

}

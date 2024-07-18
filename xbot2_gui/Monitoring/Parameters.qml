import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main

import "Parameters.js" as Logic

Item {

    id: root

    property ClientEndpoint client

    property Component boolDelegate: CheckBox {
        readonly property bool value: checked
    }

    property Component intDelegate: SpinBox {

    }

    property Component vectorDelegate: Control {
        id: control1
        property list<real> value
        contentItem: Flow {
            spacing: 8
            Repeater {
                model: control1.value.length
                TextField {
                    padding: 4
                    required property int index
                    text: `${control1.value[index]}`
                    color: palette.active.text
                    placeholderText: `id ${index}`
                    width: 60
                    onAccepted: {
                        control1.value[index] = Number(text)
                    }
                }
            }
        }
    }

    property Component doubleDelegate: RowLayout {
        id: control
        property real min: -1
        property real max: 1
        function setValue(value) {
            console.log(`setValue ${value}`)
            slider.value = value
            spin.value = value
        }
        readonly property real value: slider.value

        Slider {
            id: slider
            Layout.fillWidth: true
            from: control.min
            to: control.max
            onMoved: spin.value = value
        }
        DoubleSpinBox1 {
            id: spin
            from: control.min
            to: control.max
            onValueModified: slider.value = value
        }
    }

    property Component discreteValuesDelegate: ComboBox {
        readonly property string value: currentText
    }

    ColumnLayout {

        anchors.fill: parent

        RowLayout{
            Layout.fillWidth: true
            Button {
                Layout.fillWidth: true
                text: 'Refresh'
                onClicked: Logic.refresh()
            }
            Button {
                Layout.fillWidth: true
                text: 'Set'
                onClicked: Logic.setParams()
            }
        }

        Repeater {

            id: repeater

            RowLayout {

                property alias name: nameLabel.text
                property alias loader: loader
                property alias checked: nameLabel.checked
                spacing: 16

                CheckBox {
                    id: nameLabel
                    text: ''
                    checked: false
                }

                Loader {
                    id: loader
                    Layout.fillWidth: true

                }

            }

        }

    }


}

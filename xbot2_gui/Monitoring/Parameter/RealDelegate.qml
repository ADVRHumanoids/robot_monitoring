import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

RowLayout {

    property real min: -1

    property real max: 1

    readonly property real value: slider.value

    signal valueChangedByUser(real value)

    function setValue(value) {
        console.log(`setValue ${value} min ${min} max ${max}`)
        slider.value = value
        spin.value = value
    }

    //
    id: control

    Slider {
        id: slider
        Layout.fillWidth: true
        from: control.min
        to: control.max
        onMoved: {
            control.valueChangedByUser(value)
            spin.value = value
        }
    }

    DoubleSpinBox1 {
        id: spin
        from: control.min
        to: control.max
        onValueModified: function(value) {
            control.valueChangedByUser(value)
            slider.value = value

        }
        adaptivePrecision: true
    }
}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

RowLayout {

    property real min: -1

    property real max: 1

    readonly property real value: slider.value

    function setValue(value) {
        console.log(`setValue ${value}`)
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
        onMoved: spin.value = value
    }

    DoubleSpinBox1 {
        id: spin
        from: control.min
        to: control.max
        onValueModified: slider.value = value
    }
}

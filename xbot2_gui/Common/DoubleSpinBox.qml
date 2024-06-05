import QtQuick
import QtQuick.Controls

Item {

    id: root

    property real from: 0.0
    property real to: 1.0
    property real stepSize: 0.1
    property real value: _range / 2.0
    property int decimals: 1
    signal valueModified(real value)

    property real _range: to - from

    implicitHeight: spinbox.implicitHeight
    implicitWidth: spinbox.implicitWidth

    SpinBox {
        id: spinbox

        anchors.fill: parent

        from: 0
        to: 1000
        value: Math.round((root.value - root.from)/root._range*to)
        stepSize: (root.stepSize / root._range)*to
        editable: true

        onValueModified: {
            let v = root.from + (value / to) * root._range
            root.valueModified(v)
        }

        // onValueChanged: {
        //     root.value =  root.from + spinbox.value / 100.0 * root._range
        // }

        validator: DoubleValidator {
            bottom: root.from
            top:  root.to
            notation: DoubleValidator.StandardNotation
            decimals: root.decimals
        }

        textFromValue: function(int_value, locale) {
            return Number(root.from + (int_value / to) * root._range).toLocaleString(locale, 'f', root.decimals)
        }

        valueFromText: function(text_real, locale) {
            return (Number.fromLocaleString(locale, text_real) - root.from)/root._range * to
        }
    }
}

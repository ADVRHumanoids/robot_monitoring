import QtQuick
import QtQuick.Controls

Item {

    id: root

    property real from: 0.0
    property real to: 1.0
    property real stepSize: 0.1
    property real value: _range / 2.0
    property int decimals: 1

    property real _range: to - from

    implicitHeight: spinbox.implicitHeight
    implicitWidth: spinbox.implicitWidth

    SpinBox {
        id: spinbox

        from: 0
        to: 100
        value: Math.round((root.value - root.from)/root._range*100)
        stepSize: (root.stepSize / root._range)*100

        onValueChanged: {
            root.value =  root.from + spinbox.value / 100.0 * root._range
        }

        validator: DoubleValidator {
            bottom: root.from
            top:  root.to
        }

        textFromValue: function(int_value, locale) {
            return Number(root.from + (int_value / 100.) * root._range).toLocaleString(locale, 'f', root.decimals)
        }

        valueFromText: function(text_real, locale) {
            return (Number.fromLocaleString(locale, text_real) - root.from)/root._range * 100
        }
    }
}

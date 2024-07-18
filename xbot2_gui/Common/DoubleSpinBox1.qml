import QtQuick
import QtQuick.Controls

Item {

    property real from: 0
    property real to: 1
    property real value: 0.5

    signal valueModified(real value)

    id: root

    implicitHeight: spinBox.implicitHeight
    implicitWidth: spinBox.implicitWidth

    onValueChanged: {
        spinBox.value = spinBox.decimalToInt(value)
    }

    SpinBox {
        id: spinBox
        from: decimalToInt(root.from)
        value: decimalToInt(root.value)
        to: decimalToInt(root.to)
        stepSize: decimalToInt(1/decimalFactor)
        editable: true
        anchors.fill: parent

        property int decimals: 2
        readonly property int decimalFactor: Math.pow(10, decimals)

        function decimalToInt(decimal) {
            return decimal * decimalFactor
        }

        onValueModified: {
            root.value = value/decimalFactor
            root.valueModified(root.value)
        }

        validator: DoubleValidator {
            bottom: Math.min(spinBox.from, spinBox.to)
            top:  Math.max(spinBox.from, spinBox.to)
            decimals: spinBox.decimals
            notation: DoubleValidator.StandardNotation
        }

        textFromValue: function(value, locale) {
            return Number(value / decimalFactor).toLocaleString(locale, 'f', spinBox.decimals)
        }

        valueFromText: function(text, locale) {
            return Math.round(Number.fromLocaleString(locale, text) * decimalFactor)
        }
    }
}

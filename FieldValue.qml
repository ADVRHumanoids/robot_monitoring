import QtQuick 2.4

FieldValueForm {
    property real value: 0.0
    text: value.toFixed(2)
}

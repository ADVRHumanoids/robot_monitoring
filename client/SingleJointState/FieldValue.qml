import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11

TextArea {

    id: fieldValueText
    property real value: 0.0

    text: value.toFixed(2)
    width: fontMetrics.maximumCharacterWidth * 6

    readOnly: true
    Layout.fillWidth: true
    Layout.leftMargin: 5
    Layout.rightMargin: 5
    background: Rectangle {
        color: "lightGrey"
        radius: 4
        border.color: "darkGrey"
    }

    FontMetrics {
        id: fontMetrics
    }
}

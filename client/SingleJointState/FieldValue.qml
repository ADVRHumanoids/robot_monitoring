import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11

Label {

    id: fieldValueText
    property real value: 0.0

    text: ' ' + value.toFixed(2)

    verticalAlignment: Text.AlignVCenter

    Layout.fillWidth: true
    Layout.leftMargin: 5
    Layout.rightMargin: 5
    Layout.minimumWidth: fontMetrics.averageCharacterWidth * 8

//    background: Rectangle {
//        color: "lightGrey"
//        radius: 4
//        border.color: "darkGrey"
//    }

    FontMetrics {
        id: fontMetrics
    }
}

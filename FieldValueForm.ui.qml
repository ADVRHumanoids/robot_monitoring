import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11

TextArea {

    id: fieldValueText

    readOnly: true
    Layout.preferredWidth: 100
    Layout.alignment: Qt.AlignRight
    background: Rectangle {
        color: "lightGrey"
        radius: 4
        border.color: "darkGrey"
    }
}

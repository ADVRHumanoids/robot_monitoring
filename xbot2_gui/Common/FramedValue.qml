import QtQuick
import QtQuick.Controls
import QtQuick.Layouts


Control {

    property alias title: titleLabel.text
    property alias value1: valueLabel1.text
    property alias value2: valueLabel2.text
    property alias text: valueLabel1.text
    property color color: 'white'
    property alias radius: backgroundRect.radius

    //
    id: root
    padding: 6

    background: Rectangle {
        id: backgroundRect
        color: Qt.alpha(root.color, 0.03)
        border.color: Qt.lighter(CommonProperties.colors.cardBackground)
        border.width: 1
        radius: 4
    }


    contentItem: ColumnLayout {

        spacing: 2

        Label {
            id: titleLabel
            text: 'Title'
            font.pixelSize: CommonProperties.font.h3
            wrapMode: Text.WrapAnywhere
            Layout.fillWidth: true
            color: root.color
        }

        Item {
            Layout.preferredHeight: 2
            Layout.fillHeight: true
        }

        Label {
            id: valueLabel1
            visible: text !== ''
            font.bold: true
            wrapMode: Text.Wrap
            Layout.fillWidth: true
            color: root.color
        }

        Label {
            id: valueLabel2
            visible: text !== ''
            font.bold: true
            wrapMode: Text.Wrap
            Layout.fillWidth: true
            color: root.color
        }
    }

}

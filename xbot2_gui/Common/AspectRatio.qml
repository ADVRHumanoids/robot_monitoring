import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {

    id: root

    property real aspectRatio: 1.0

    property real _ar: width/height

    default property alias content: inner.children

    Rectangle {
        id: inner
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        width: root._ar < root.aspectRatio ? parent.width : parent.height*root.aspectRatio
        height: root._ar < root.aspectRatio ? parent.width/root.aspectRatio : parent.height

        color: 'transparent'
        border {
            color: 'red'
            width: 1
        }
    }
}

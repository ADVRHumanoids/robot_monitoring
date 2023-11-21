import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Shapes 6.0


Item {
    width: 100
    height: 400

    property string fillColor: "green"
    property string strokeColor: "red"
    antialiasing: true
    property int strokeWidth: -1

    TriangleItem {
        id: triangleUp

        radius: 0
        strokeWidth: parent.strokeWidth
        rotation: 180

        anchors.top: parent.top
        width: parent.width
        height: parent.height * 0.4

        fillColor: parent.fillColor
        strokeColor: parent.strokeColor
        antialiasing: parent.antialiasing
    }

    TriangleItem {
        id: triangleDown

        radius: 0
        strokeWidth: parent.strokeWidth

        anchors.bottom: parent.bottom
        width: parent.width
        height: parent.height * 0.4

        fillColor: parent.fillColor
        strokeColor: parent.strokeColor
        antialiasing: parent.antialiasing
    }
}

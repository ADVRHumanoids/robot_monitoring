import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts

RowLayout {

    property string jointName: "joint_name"
    property alias bar: bar
    property real labelColorAlpha: 0.3
    property alias labelMouseArea: labelMouseArea
    property bool statusOk: false

    width: 400
    height: 80

    Label {
        id: jointNameLabel
        text: jointName
        background: Rectangle {
            border.color: "red"
            color: statusOk ? Qt.rgba(0.9, 0.9, 0.9, labelColorAlpha) : Qt.rgba(1, 0, 0, labelColorAlpha)
            radius: 3
            border.width: statusOk ? 0 : 1
        }
        font.pixelSize: Qt.application.font.pixelSize * 0.9
        wrapMode: Text.Wrap
        Layout.preferredWidth: 100
        padding: 5

        MouseArea {
            id: labelMouseArea
            anchors.fill: parent
            hoverEnabled: true
        }
    }

    TwoSideBar {
        id: bar
        Layout.fillWidth: true
        height: jointNameLabel.height
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}D{i:1}D{i:3}
}
##^##*/


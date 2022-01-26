import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts

RowLayout {

    property string jointName: "joint_name"
    property alias bar: bar
    property bool statusOk: false

    width: 400
    height: 80

    Label {
        id: jointNameLabel
        text: jointName
        background: Rectangle {
            border.color: "red"
            color: Qt.rgba(1, 0, 0, 0.3)
            radius: 3
            opacity: statusOk ? 0 : 1
        }
        Layout.preferredWidth: 100
        padding: 5
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


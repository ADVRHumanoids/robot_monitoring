import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11

Item {

    property alias grid: grid
    implicitHeight: grid.implicitHeight + label.height
    implicitWidth: grid.implicitWidth

    Label {
        id: label
        text: singleJointState.jName + " (ID: " + singleJointState.jId + ")"
        font.pixelSize: 20
        bottomPadding: 10
    }

    GridLayout {

        id: grid
        columns: 3
        anchors {
            top: label.bottom
            bottom: parent.bottom
            left: parent.left
            right: parent.right
        }

        rowSpacing: -6
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/


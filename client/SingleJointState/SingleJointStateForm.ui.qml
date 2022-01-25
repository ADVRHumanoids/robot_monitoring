import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11

GroupBox {

    property alias grid: grid

    title: singleJointState.jName + " (ID: " + singleJointState.jId + ")"

    GridLayout {

        id: grid
        columns: 3
        anchors.fill: parent
        rowSpacing: 0
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/


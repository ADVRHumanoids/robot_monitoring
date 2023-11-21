import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11

Item {

    property alias grid: grid
    implicitHeight: grid.implicitHeight
    implicitWidth: grid.implicitWidth

    GridLayout {

        id: grid
        columns: 3
        anchors.fill: parent
        rowSpacing: 0

        onImplicitHeightChanged: {
            console.log(`diocane ${implicitHeight}`)
        }
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/


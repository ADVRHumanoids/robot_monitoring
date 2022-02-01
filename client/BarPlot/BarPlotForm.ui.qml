import QtQuick 2.4
import QtQuick.Layouts
import QtQuick.Controls 2.12

ScrollView {

    property alias container: container
    property int type: TwoSideBar.Type.Bar

    id: scroller
    clip : true
    contentWidth: availableWidth
    contentHeight: Math.max(grid.implicitHeight, parent.height)

    GridLayout
    {
        id: grid
        anchors.fill: parent
        rowSpacing: 2
        columnSpacing: 10
        flow: GridLayout.TopToBottom

        readonly property int elementWidth: 200

        columns: Math.max(Math.floor(parent.width / elementWidth), 1)
        rows: Math.max(Math.ceil(children.length / columns), 1)

        Repeater {
            id: container
            model: root.jointNames.length
            BarPlotItem {
                Layout.fillWidth: true
                bar.min: root.min[index]
                bar.max: root.max[index]
                bar.value: 0
                jointName: root.jointNames[index]
                bar.type: type

                onJointClicked: function(jn) {
                    root.jointClicked(jn)
                }
            }
        }
    }

}

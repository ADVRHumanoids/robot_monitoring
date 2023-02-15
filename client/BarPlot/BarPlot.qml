import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import "logic.js" as Logic

// allow to scroll this widget if viewport height
// is too small
Item {

    // main properties
    property var jointNames: []
    property var min: []
    property var max: []
    property string fieldName: "tor"
    property string fieldNameRef: "torRef"
    property alias container: container
    property int type: TwoSideBar.Type.Bar

    function setStatus(ok) {
        console.log(ok)
        console.log(ok.length)
        console.log(container.count)
        for(let idx = 0; idx < container.count; idx++) {
            container.itemAt(idx).statusOk = ok[idx]
        }
    }

    function setJointStateMessage(js_msg) {
        Logic.setJointStateMessage(js_msg)
    }

    signal jointClicked(string jname)

//    implicitWidth: grid.elementWidth
    implicitHeight: grid.implicitHeight

    id: root
//    clip : true
//    contentWidth: availableWidth
//    contentHeight: Math.max(grid.implicitHeight, parent.height)

    GridLayout
    {
        id: grid
        anchors.fill: parent
        rowSpacing: 4
        columnSpacing: 10
        flow: GridLayout.TopToBottom

        readonly property int elementWidth: 220

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

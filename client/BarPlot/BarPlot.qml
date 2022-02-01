import QtQuick 2.4
import "logic.js" as Logic

BarPlotForm {

    id: root

    property var jointNames: []
    property var min: []
    property var max: []
    property string fieldName: "tor"
    property string fieldNameRef: "torRef"

    function setJointStateMessage(js_msg) {
        Logic.setJointStateMessage(js_msg)
    }

    signal jointClicked(string jname)

}

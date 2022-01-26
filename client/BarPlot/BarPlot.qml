import QtQuick 2.4
import "/SingleJointState/data.js" as Data
import "logic.js" as Logic

BarPlotForm {

    id: root

    property var jointNames: []
    property var min: []
    property var max: []
    property string fieldName: "motPos"

    function setJointStateMessage(js_msg) {
        Logic.setJointStateMessage(js_msg)
    }

}

import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts

import "data.js" as Data
import "../TestThings"

SingleJointStateForm {

    id: singleJointState

    property var labelComponent: Component {
        Label {

        }
    }

    property var auxSelectorComponent: Component {
        AuxSelector {
            Layout.fillHeight: true
        }
    }

    property var valueComponent: Component {
        FieldValue {
        }
    }

    property var sepComponent: Component {
        Separator { }
    }

    property var plotBtnComponent: Component {
        ToolButton {
            text: 'Plot'
            Layout.preferredHeight: implicitHeight - 12
            topPadding: 0
            bottomPadding: 0
        }
    }

    property string jName: "joint_name"
    property int jId: 666
    property int jIndex: 0

    property var fieldValueMap: Object()
    property var plotBtnMap: Object()

    signal plotAdded(string jName, string fieldName)
    signal plotRemoved(string jName, string fieldName)

    function setJointStateMessage(msg) {
        Data.setJointStateMessage(msg)
    }

    function setFaultCode(faultCode) {
        let faultValue = fieldValueMap['fault']
        faultValue.setText(faultCode)
        faultValue.alert = true
    }

    Component.onCompleted: {
        Data.buildFields(grid)
    }
}

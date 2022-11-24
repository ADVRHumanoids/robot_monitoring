import QtQuick 2.4
import QtQuick.Controls 2.15
import QtQuick.Layouts

import "data.js" as Data

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
        Switch { }
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

    Component.onCompleted: {
        Data.buildFields(grid)
    }
}

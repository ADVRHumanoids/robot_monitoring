import QtQuick 2.4
import QtQuick.Controls 2.15
import "../sharedData.js" as SharedData

SingleJointStateStackForm {

    id: root

    property int currentIndex: 0

    function selectJoint(jointName)
    {
        currentIndex = SharedData.jointNames.indexOf(jointName)
    }

    property var jointStateComponent: Component {
        SingleJointState {

        }
    }

    property var jointNames: []

    function setJointStateMessage(msg) {
        for(var i = 0; i < loader.count; i++)
        {
            let item = loader.itemAt(i).item

            if(item === null)
            {
                continue
            }

            item.setJointStateMessage(msg)
        }
    }

    function construct() {
        jointNames = SharedData.jointNames
    }
}

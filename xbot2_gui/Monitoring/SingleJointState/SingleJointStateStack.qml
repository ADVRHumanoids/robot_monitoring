import QtQuick
import QtQuick.Controls

import "/qt/qml/Main/sharedData.js" as SharedData

SingleJointStateStackForm {

    id: root

    signal plotAdded(string jName, string fieldName)
    signal plotRemoved(string jName, string fieldName)

    property list<string> auxFieldNames

    property int currentIndex: 0

    function selectJoint(jointName)
    {
        currentIndex = SharedData.jointNames.indexOf(jointName)
    }

    property var jointStateComponent: Component {
        SingleJointState {
            auxFieldNames: root.auxFieldNames
        }
    }

    property var jointNames: SharedData.jointNames

    function setJointStateMessage(msg) {

        for(let at of msg.aux_types) {
            if(root.auxFieldNames.indexOf(at) < 0)
            {
                auxFieldNames.push(at)
            }
        }

        let item = repeater.itemAt(currentIndex).item

        if(item === null)
        {
            return
        }

        item.setJointStateMessage(msg)

    }

    function setFaultCode(jName, faultCode) {

        let item = repeater.itemAt(currentIndex).item

        if(item === null)
        {
            return
        }

        if(item.jName === jName) {
            item.setFaultCode(faultCode)
        }
    }
}

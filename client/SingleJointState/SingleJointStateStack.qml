import QtQuick 2.4
import QtQuick.Controls 2.15
import "../sharedData.js" as SharedData

SingleJointStateStackForm {

    id: root

    signal plotAdded(string jName, string fieldName)
    signal plotRemoved(string jName, string fieldName)

    property int currentIndex: 0

    function selectJoint(jointName)
    {
        currentIndex = SharedData.jointNames.indexOf(jointName)
    }

    property var jointStateComponent: Component {
        SingleJointState {

        }
    }

    property var jointNames: SharedData.jointNames

    function setJointStateMessage(msg) {

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

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

    signal addJoint(var name, var id)

    signal progressChanged(var msg)

    signal constructionCompleted()

    function setJointStateMessage(msg) {
        for(var i = 0; i < loader.count; i++)
        {
            loader.itemAt(i).item.setJointStateMessage(msg)
        }
    }

    onAddJoint: function (name, id) {
        var obj = jointStateComponent.createObject(stack, {jName: name, jId: id})
        print('[joint state] ' + name + ' added')
        items.push(obj)
        progressChanged('[joint state] ' + name + ' added')
        appData.updateUi()
    }

    function construct() {
        jointNames = SharedData.jointNames
        loader.model = [{jName: jointNames[0]}]
    }

    onCurrentIndexChanged: {

        var reqestedName = SharedData.jointNames[currentIndex]

        var mdlIndex = loader.model.findIndex((elem) =>
            elem.jName === reqestedName
        )

        if(mdlIndex === -1) {
            print('not found ' + reqestedName)
            var mdl = loader.model
            mdl.push({jName: reqestedName})
            loader.model = mdl
            mdlIndex = mdl.length - 1
        }

        print('set index ' + mdlIndex)
        stack.currentIndex = mdlIndex

    }
}

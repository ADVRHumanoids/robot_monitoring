import QtQuick 2.4
import QtQuick.Controls 2.15

SingleJointStateStackForm {

    id: jointStateStack

    property int currentIndex: 0

    property var jointStateComponent: Component {
        SingleJointState {

        }
    }

    property var items: []

    signal addJoint(var name, var id)

    signal progressChanged(var msg)

    signal constructionCompleted()

    function setJointStateMessage(msg) {
        for(var i = 0; i < items.length; i++)
        {
            items[i].setJointStateMessage(msg)
        }
    }

    onAddJoint: function (name, id) {
        var obj = jointStateComponent.createObject(stack, {jName: name, jId: id})
        print('[joint state] ' + name + ' added')
        items.push(obj)
        progressChanged('[joint state] ' + name + ' added')
        appData.updateUi()
    }

    function construct(names) {

        // connect completed signal
        progressChanged.connect( function (msg) {
            if(items.length === names.length)
            {
                constructionCompleted()
            }
        } )

        // trigger all signals
        for(var i = 0; i < names.length; i++)
        {
            addJoint(names[i], i)
        }


    }
}

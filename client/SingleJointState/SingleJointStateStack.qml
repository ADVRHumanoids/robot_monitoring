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

    function setJointStateMessage(msg) {
        for(var i = 0; i < items.length; i++)
        {
            items[i].setJointStateMessage(msg)
        }
    }

    function addJoint(name: string, id: int) {
        var obj = jointStateComponent.createObject(stack, {jName: name, jId: id})
        items.push(obj)
        print('adding joint ' + name)
    }

    function construct(names) {
        print('constructing with ' + names)
        for(var i = 0; i < names.length; i++)
        {
            addJoint(names[i], i)
        }
    }
}

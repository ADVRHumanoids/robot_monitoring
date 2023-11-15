import QtQuick
import QtQuick.Controls

NavBar {

    property var model: []

    signal hamburgerClicked()

    //
    id: root

    property Component barButtonComponent: Component {
        NavButton {
            checkedDisplayMode: AbstractButton.TextBesideIcon
            uncheckedDisplayMode: AbstractButton.IconOnly
        }
    }

    Component.onCompleted: {
        for(let i = 0; i < model.children.length; i++) {
            let obj = model.children[i]
            barButtonComponent.createObject(root, {'text': obj.name, 'iconChar': obj.iconText})
        }
    }
}

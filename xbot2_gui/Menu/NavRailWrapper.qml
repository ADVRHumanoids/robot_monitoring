import QtQuick
import QtQuick.Controls

Item {

    property var model: []

    property int currentIndex: 0

    signal hamburgerClicked()

    //
    id: root


    property Component barButtonComponent: Component {
        Item {
            width: btn.width
            height: btn.height
            anchors.horizontalCenter: parent.horizontalCenter
            property string iconChar
            property string text
            property int index
            NavButton {
                id: btn
                checkedDisplayMode: AbstractButton.TextUnderIcon
                uncheckedDisplayMode: AbstractButton.TextUnderIcon
                textFixedWidth: 50
                anchors.centerIn: parent
                iconChar: parent.iconChar
                text: parent.text
                checked: root.currentIndex === parent.index
                onClicked: root.currentIndex = parent.index
            }
        }
    }

    Component.onCompleted: {
        for(let i = 0; i < model.children.length; i++) {
            let obj = model.children[i]
            let btn = barButtonComponent.createObject(col, {'text': obj.name, 'iconChar': obj.iconText, 'index': i})
        }
    }

    ScrollView {
        id: scroll
        width: parent.width
        height: Math.min(contentHeight, parent.height - 100)
        anchors.centerIn: parent
        contentWidth: availableWidth
        ScrollBar.vertical.policy: ScrollBar.AlwaysOff

        Column {
            id: col
            width: scroll.availableWidth
            spacing: 5
        }

    }


}

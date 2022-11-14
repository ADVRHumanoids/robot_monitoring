import QtQuick
import QtQuick.Controls

Rectangle {

    id: root
    color: "blue"

    property var model: []
    property int currentIndex: 0

    signal hamburgerClicked()

    Hamburger {
        id: ham
        anchors {
            right: parent.right
            verticalCenter: parent.verticalCenter
            margins: 6
        }

        height: parent.height - 12
        width: height*1.2

        onClicked: {
            hamburgerClicked()
        }
    }

    Row {

        id: row
        spacing: 16
        anchors.centerIn: parent

        Component.onCompleted: {
            for(let i = 0; i < model.count; i++) {
                let obj = model.get(i)
                let icon = iconComponent.createObject(row, {'name': obj.name, 'index': i})
            }
        }

        Component {
            id: iconComponent
            NavIcon {
                property int index: -1
                width: ham.width
                isSelected: currentIndex === index

                onClicked: {
                    currentIndex = index
                }
            }
        }
    }



}

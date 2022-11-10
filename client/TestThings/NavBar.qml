import QtQuick
import QtQuick.Controls

Rectangle {

    color: "blue"

    property var model: []

    signal hamburgerClicked()

    Hamburger {
        id: ham
        anchors {
            right: parent.right
            verticalCenter: parent.verticalCenter
            margins: 6
        }

        height: parent.height - 12
        width: height

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
                iconComponent.createObject(row, {'text': obj.name[0]})
            }
        }

        Component {
            id: iconComponent
            Label {
                font.pixelSize: 24
                width: ham.width
            }
        }
    }



}

import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

RailMenu {

    id: root
    color: "blue"
    railWidth: ham.width + 2*16
    menuWidth: 300

    property var model: []

    GridLayout {

        id: grid

        columns: 2

        anchors {
            verticalCenter: parent.verticalCenter
            horizontalCenter: parent.horizontalCenter
        }

        width: parent.width - 32

        Component.onCompleted: {

            for(let i = 0; i < model.count; i++) {
                let obj = model.get(i)
                nameComponent.createObject(grid, {'text': obj.name})
                iconComponent.createObject(grid, {'text': obj.name[0]})
            }
        }

        Component {
            id: nameComponent
            Label {
                Layout.fillWidth: true
                font.pixelSize: 20
            }
        }

        Component {
            id: iconComponent
            Label {
                font.pixelSize: 32
                width: ham.width
            }
        }


    }

    Item {

        id: header

        height: 40

        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
            margins: 16
        }

        Label {
            id: guiNameLabel
            text: 'xbot2 gui'
            font.pixelSize: Qt.application.font.pixelSize * 2
            font.bold: true
            anchors {
                left: parent.left
                verticalCenter: parent.verticalCenter
            }
        }

        Hamburger {
            id: ham
            width: 40
            height: guiNameLabel.height
            anchors {
                right: parent.right
                verticalCenter: parent.verticalCenter
            }

            onClicked: {
                if(root.position > 0.5) {
                    root.close()
                }
                else {
                    root.open()
                }
            }
        }


    }

}

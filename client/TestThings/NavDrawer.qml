import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

RailMenu {

    id: root
    color: Material.primaryColor
    railWidth: ham.width + 2*16
    menuWidth: 300
    property int currentIndex: 0

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
                let nameObj = nameComponent.createObject(grid, {'text': obj.name})
                let iconObj = iconComponent.createObject(grid, {'text': obj.name[0]})
                nameObj.clicked.connect(function(){
                    root.currentIndex = i
                    root.close()
                })
                iconObj.clicked.connect(function(){
                    root.currentIndex = i
                    root.close()
                })
            }
        }

        Component {
            id: nameComponent
            Label {
                signal clicked()
                Layout.fillWidth: true
                font.pixelSize: 20
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        parent.clicked()
                    }
                }
            }
        }

        Component {
            id: iconComponent
            Label {
                signal clicked()
                font.pixelSize: 32
                width: ham.width
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        parent.clicked()
                    }
                }
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

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

RailMenu {

    id: root
    color: CommonProperties.colors.primary
    railWidth: ham.width + 2*16
    menuWidth: 300
    property int currentIndex: 0

    property Item model: []

    GridLayout {

        id: grid

        columns: 2

        anchors {
            verticalCenter: parent.verticalCenter
            horizontalCenter: parent.horizontalCenter
        }

        width: parent.width - 32

        Component.onCompleted: {

            for(let i = 0; i < model.children.length; i++) {
                let obj = model.children[i]
                let nameObj = nameComponent.createObject(grid, {'text': obj.name})
                let iconObj = iconComponent.createObject(grid, {'name': obj.name[0], 'index': i})
                nameObj.clicked.connect(function(){
                    root.currentIndex = i
                    root.close()
                })
                nameObj.enabled = Qt.binding(() => {return obj.active})
                iconObj.enabled = Qt.binding(() => {return obj.active})
            }
        }

        Component {
            id: nameComponent
            Label {
                signal clicked()
                Layout.fillWidth: true
                font.pixelSize: CommonProperties.font.h2
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
            NavIcon {
                property int index: -1
                Layout.preferredWidth: ham.width
                isSelected: root.currentIndex === index
                onClicked: {
                    root.currentIndex = index
                    root.close()
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
            font.pixelSize: CommonProperties.font.h1
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

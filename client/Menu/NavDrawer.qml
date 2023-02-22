import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.Common

RailMenu {

    readonly property int implicitRailWidth: ham.implicitWidth + 2*margins

    property int currentIndex: 0

    property Item model: []


    // private
    id: root
    color: CommonProperties.colors.primary
    railWidth: 20
    menuWidth: 300
    property int margins: 8


    ScrollView {

        id: scroll

        anchors {
            left: parent.left
            right: parent.right
        }

        anchors.margins: root.margins

        y: Math.max(header.y + header.height + 26,
                    root.height/2. - height/2.)

        height: Math.min(contentHeight,
                         root.height - header.height - 32)

        ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
        ScrollBar.vertical.policy: ScrollBar.AlwaysOff

        GridLayout {

            id: grid

            columns: 2

            width: scroll.availableWidth

            Component.onCompleted: {

                for(let i = 0; i < model.children.length; i++) {
                    let obj = model.children[i]
                    let nameObj = nameComponent.createObject(grid, {'text': obj.name})
                    let iconObj = iconComponent.createObject(grid, {
                                                                 'name': obj.iconText,
                                                                 'index': i,
                                                                 'fontFamily': obj.iconFont.family
                                                             })
                    nameObj.clicked.connect(function(){
                        root.currentIndex = i
                        root.close()
                    })
                    nameObj.enabled = Qt.binding(() => {return obj.active})
                    iconObj.enabled = Qt.binding(() => {return obj.active})
                }
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
            margins: root.margins
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

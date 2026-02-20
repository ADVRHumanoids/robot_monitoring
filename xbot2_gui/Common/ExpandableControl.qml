import QtQuick
import QtQuick.Controls
import QtQuick.Layouts


Control {

    default property alias dfl: contentWrapper.children

    //
    id: root

    property bool collapsed: true

    background: Rectangle {
        radius: 6
        color: Qt.rgba(0, 0, 0, 0.1)
    }

    contentItem: ColumnLayout {

        id: colLayout

        // header
        RowLayout {

            Layout.fillWidth: true

            Label {
                text: 'My Title'
                font.pointSize: 14
                Layout.fillWidth: !root.collapsed
                Layout.preferredHeight: root.collapsed ? 0 : implicitHeight
                Layout.preferredWidth: root.collapsed ? 0 : implicitWidth
                clip: true

                Behavior on Layout.preferredHeight {
                    NumberAnimation {
                        duration: 333
                        easing.type: Easing.OutQuad
                    }
                }

                Behavior on Layout.preferredWidth {
                    NumberAnimation {
                        duration: 333
                        easing.type: Easing.OutQuad
                    }
                }
            }

            Item {
                Layout.fillWidth: root.collapsed
                visible: !root.collapsed
            }

            RoundButton {
                text: root.collapsed ? 'O' : 'X'
                onClicked: root.collapsed = !root.collapsed
            }

        }

        // content
        Item {
            id: contentWrapper
            Layout.fillHeight: !root.collapsed
            Layout.fillWidth: !root.collapsed
            Layout.preferredHeight: root.collapsed ? 0 : implicitHeight
            Layout.preferredWidth: root.collapsed ? 0 : implicitWidth
            implicitHeight: children[0].implicitHeight
            implicitWidth: children[0].implicitWidth
            clip: true

            Behavior on Layout.preferredHeight {
                NumberAnimation {
                    duration: 333
                    easing.type: Easing.OutQuad
                }
            }

            Behavior on Layout.preferredWidth {
                NumberAnimation {
                    duration: 333
                    easing.type: Easing.OutQuad
                }
            }
        }


    }

    // Behavior on implicitHeight {
    //     NumberAnimation {
    //         duration: 333
    //         easing.type: Easing.OutQuad
    //     }
    // }

    // Behavior on implicitWidth {
    //     NumberAnimation {
    //         duration: 333
    //         easing.type: Easing.OutQuad
    //     }
    // }

}

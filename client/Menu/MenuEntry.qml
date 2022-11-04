import QtQuick

Item {

    id: root

    property color color: Qt.rgba(0, 0, 0, 0)

    signal released()

    width: text.implicitWidth
    height: text.implicitHeight

    Rectangle {

        id: background
        anchors.fill: parent
        color: root.color

        Behavior on color {

            ColorAnimation {
                duration: 600
            }
        }

        Text {
            id: text
            topPadding: 10
            bottomPadding: 10
            text: name
            color: root.enabled ? "white" : "grey"
            font.pixelSize: Qt.application.font.pixelSize * 2

            Behavior on leftPadding {
                NumberAnimation {
                    duration: 300
                }
            }
        }

        Image {
            id: icon
            source: '../Icons/home_dark.png'
            anchors {
                top: parent.top
                right: parent.right
                bottom: parent.bottom
                margins: 5
            }
            width: height
        }

//        ShaderEffect {
//            property variant src: icon
//            fragmentShader: '/Icons/shader/invert.frag.qsb'
//        }

    }

    MouseArea {
        enabled: root.enabled
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true

        onHoveredChanged: {
            background.color = containsMouse ? Qt.darker(root.color) : root.color
            text.leftPadding = containsMouse ? 10 : 0
        }

        onReleased: root.released()
    }

}

import QtQuick
import QtQuick.Controls

BusyIndicator {
    id: loader
    property color loaderColor: "#91d7e3"
    contentItem: Item {
        implicitWidth: 200
        implicitHeight: 200

        Item {
            id: item
            // x: parent.width / 2 - (50)
            // y: parent.height / 2 - (50)
            width: parent.width
            height: parent.width

            opacity: loader.running ? 1 : 0

            Behavior on opacity {
                OpacityAnimator {
                    duration: 250
                }
            }

            RotationAnimator {
                target: item
                running: loader.visible && loader.running
                from: 0
                to: 360
                loops: Animation.Infinite
                duration: 1750
            }

            Repeater {
                id: repeater
                model: 6

                Rectangle {
                    id: delegate
                    x: item.width / 2 - width / 2
                    y: item.height / 2 - height / 2
                    implicitWidth: 24
                    implicitHeight: 24
                    radius: 200
                    color: loaderColor

                    required property int index

                    transform: [
                        Translate {
                            y: -Math.min(item.width, item.height) * 0.5 + 5
                        },
                        Rotation {
                            angle: delegate.index / repeater.count * 360
                            origin.x: delegate.implicitWidth / 2
                            origin.y: delegate.implicitHeight / 2
                        }
                    ]
                }
            }
        }
    }
}


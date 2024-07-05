import QtQuick
import QtQuick.Controls


BusyIndicator {
    id: taskBusyIndicator
    contentItem: Item {
        implicitWidth: 50
        implicitHeight: 50

        Item {
            id: item
            x: parent.width / 2 - 25
            y: parent.height / 2 - 25
            width: 50
            height: 50
            opacity: taskBusyIndicator.running ? 1 : 0

            Behavior on opacity {
                OpacityAnimator {
                    duration: 250
                }
            }

            RotationAnimator {
                target: item
                running: taskBusyIndicator.visible && taskBusyIndicator.running
                from: 0
                to: 360
                loops: Animation.Infinite
                duration: 1500
            }

            Repeater {
                id: repeater
                model: 6

                Rectangle {
                    id: delegate
                    x: item.width / 2 - width / 2
                    y: item.height / 2 - height / 2
                    implicitWidth: 8
                    implicitHeight: 8
                    radius: 5
                    color: "#8bd5ca"

                    required property int index

                    transform: [
                        Translate {
                            y: -Math.min(item.width, item.height) * 0.5 + 5
                        },
                        Rotation {
                            angle: delegate.index / repeater.count * 360
                            origin.x: 5
                            origin.y: 5
                        }
                    ]
                }
            }
        }
    }
}


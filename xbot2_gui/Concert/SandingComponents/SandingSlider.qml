import QtQuick
import QtQuick.Controls.Basic

Item {
    id: root
    property real minValue: 0.0
    property real maxValue: 1.0
    property real step: 0.1
    property real defaultValue: 0.5

    signal valueSet(real v)

    implicitHeight: control.implicitHeight
    implicitWidth: control.implicitWidth

    Slider {
        id: control
        from: minValue
        to: maxValue
        value: defaultValue
        stepSize: step

        background: Rectangle {
            x: control.leftPadding
            y: control.topPadding + control.availableHeight / 2 - height / 2
            implicitWidth: 200
            implicitHeight: 4
            width: control.availableWidth
            height: implicitHeight
            radius: 2
            color: "#cdd6f4"

            Rectangle {
                width: control.visualPosition * parent.width
                height: parent.height
                color: "#8bd5ca"
                radius: 2
            }
        }

        handle: Rectangle {
            x: control.leftPadding + control.visualPosition * (control.availableWidth - width)
            y: control.topPadding + control.availableHeight / 2 - height / 2
            implicitWidth: 20
            implicitHeight: 20
            radius: 13
            color: control.pressed ? "#b8c0e0" : "#cdd6f4"
            border.color: "#8bd5ca"
        }
        onMoved: {
            root.valueSet(value)
        }
    }
}


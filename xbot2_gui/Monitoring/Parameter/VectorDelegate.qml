import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Control {

    property var value: {
        'data': []
    }

    //
    id: control

    contentItem: Flow {

        spacing: 8

        Repeater {

            model: control.value.data.length

            TextField {
                padding: 4
                required property int index
                text: `${control.value.data[index]}`
                color: palette.active.text
                placeholderText: `id ${index}`
                width: 60
                onEditingFinished: {
                    control.value.data[index] = Number(text)
                }
            }

        }
    }
}

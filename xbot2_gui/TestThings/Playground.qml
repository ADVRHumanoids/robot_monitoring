import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebView

import Common
import Main
import ExpandableBottomBar
import Font
import Menu
import Joy

Item {

    property ClientEndpoint client

    id: root

    MultiColumnLayout1 {
        anchors.fill: parent

        columns: Math.ceil(width / 300)

        Repeater {

            model: spin.value

            AnimatedRectangle {
                required property int index
                color: 'green'
                width: 200
                height: 200
                Text {
                    anchors.centerIn: parent
                    text: `${index}/${spin.value}`
                }
            }

        }
    }

    SpinBox {
        id: spin
        from: 0
        to: 20
        value: 10
    }
}

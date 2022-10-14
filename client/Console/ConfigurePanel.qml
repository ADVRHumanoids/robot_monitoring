import QtQuick 2.15
import QtQuick.Layouts
import QtQuick.Controls

import "configure_panel.js" as Logic

Rectangle {

    property bool hidden: true

    property var description: Object()

    property alias _grid: mainLayout

    id: root
    height: parent.height
    width: 300
    x: hidden ? parent.width : parent.width - width
    color: 'red'
    z: 1

    Behavior on x {
        NumberAnimation {
            duration: 500
            easing.type: Easing.OutQuad
        }
    }

    MouseArea {
        anchors.right: parent.left
        enabled: !parent.hidden
        height: parent.height
        width: 200

        onReleased: {
            parent.hidden = true
        }
    }

    // content

    GridLayout {
        id: mainLayout
        anchors.fill: parent
        columns: 2

        Label {
            text: 'prova 1'
        }

        CheckBox {
            text: ''
        }
    }

    property var label: Component {

        Label {

        }

    }

    property var combo: Component {

        ComboBox {

        }

    }

    property var check: Component {

        CheckBox {

        }

    }

    // initialization
    onDescriptionChanged: {
        Logic.construct()
        console.log(mainLayout.children.length)
    }

}

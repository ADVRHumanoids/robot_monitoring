import QtQuick 2.15
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Controls.Material
import Qt.labs.settings

import "configure_panel.js" as Logic

Rectangle {

    property bool hidden: true

    property var description: Object()

    property var options: Object()

    property var _controls: Object()
    property alias _grid: mainLayout
    property alias _cancelBtn: cancelBtn
    property alias _okBtn: okBtn

    id: root
    height: parent.height
    width: Math.min(300, parent.width)
    x: hidden ? parent.width : parent.width - width
    color: Material.primary

    Behavior on x {
        NumberAnimation {
            duration: 500
            easing.type: Easing.OutQuad
        }
    }

    // content

    ScrollView {

        anchors.fill: parent

        // layout to be filled by Logic.construct()
        GridLayout {
            id: mainLayout
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: 10
            columns: 2
            columnSpacing: 20
        }

    }

    // will be reparented to mainLayout
    // upon Logic.construct()
    Button {
        id: cancelBtn
        text: "Cancel"
        onReleased: {
            hidden = true
        }
    }

    // will be reparented to mainLayout
    // upon Logic.construct()
    Button {
        id: okBtn
        text: "Ok"
        onReleased: {
            Logic.apply()
            hidden = true
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

    property var text: Component {

        TextArea {

        }

    }

    // initialization
    onDescriptionChanged: {
        Logic.construct()
    }

}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import xbot2_gui.common
import Qt.labs.settings

import "configure_panel.js" as Logic
import "../TestThings"

Item {

    // public

    property var description: Object()

    property var options: Object()

    function applySettings() {
        Logic.apply()
    }

    // private

    id: root

    property var _controls: Object()
    property alias _grid: mainLayout

    implicitHeight: mainLayout.implicitHeight
    implicitWidth: mainLayout.implicitWidth


    // layout to be filled by Logic.construct()
    GridLayout {
        id: mainLayout
        anchors.fill: parent
        columns: 2
        columnSpacing: 16
    }

    // will be reparented to mainLayout
    // upon Logic.construct()
    //    Button {
    //        id: cancelBtn
    //        text: "Cancel"
    //        onReleased: {
    //            hidden = true
    //            closeRequested()
    //        }
    //    }

    //    // will be reparented to mainLayout
    //    // upon Logic.construct()
    //    Button {
    //        id: okBtn
    //        text: "Ok"
    //        onReleased: {
    //            Logic.apply()
    //            hidden = true
    //            closeRequested()
    //        }
    //    }

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

    Settings {
        id: settings
    }

    // initialization
    onDescriptionChanged: {
        Logic.construct()
    }

}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import xbot2_gui.common
import QtCore

import "ConfigurePanel.js" as Logic
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

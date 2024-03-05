import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Common

import "../Common"
import "ConsoleCard.js" as Logic

Item {

    property string name

    property bool scrollOnOutput: true

    function appendText(text) {

        model.append({'txt': text})

        if(scrollOnOutput) view.positionViewAtEnd()
    }

    function clearText() {
        model.clear()
    }

    id: root
    implicitHeight: view.implicitHeight
    implicitWidth: view.implicitWidth

    property Component delegate: Component {
        TextEdit {

            MouseArea {
                anchors.fill: parent
                z: 1
            }

            // placeholderText: name
            font.pixelSize: 14
            width: view.width
            wrapMode: Text.WrapAnywhere
            readOnly: true
            textFormat: TextEdit.RichText
            text: txt
            // visible: level >= root.verbosity
            color: palette.text
            // property list<color> levelToColor: [palette.text, CommonProperties.colors.warn, CommonProperties.colors.err]
        }
    }

    ListModel {
        id: model
    }

    ListView {
        id: view
        model: model
        delegate: root.delegate
        implicitHeight: contentHeight
        anchors.fill: parent
        spacing: 1
        clip: true
        ScrollBar.vertical: ScrollBar {
            active: true
        }
    }

}

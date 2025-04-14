import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Common

import "../Common"
import "ConsoleCard.js" as Logic

Item {

    property alias listModel: view.model

    function scrollToEnd() {
        Qt.callLater(view.positionViewAtEnd)
    }

    id: root
    implicitHeight: view.implicitHeight
    implicitWidth: view.implicitWidth

    property Component delegate: TextEdit {
        required property string txt
        // required property color txtColor
        font.pixelSize: 14
        width: view.width
        wrapMode: Text.WrapAnywhere
        readOnly: true
        textFormat: TextEdit.RichText
        text: txt
        // color: txtColor
    }

    ListView {
        id: view
        delegate: root.delegate
        // implicitHeight: contentHeight  // note: breaks performance!!!!
        anchors.fill: parent
        spacing: 1
        clip: true
        ScrollBar.vertical: ScrollBar {
            active: true
        }
    }
}

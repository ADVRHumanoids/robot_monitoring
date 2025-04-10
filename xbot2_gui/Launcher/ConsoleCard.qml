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

        if(scrollOnOutput) Qt.callLater(view.positionViewAtEnd)
    }

    function clearText() {
        model.clear()
    }

    function getText() {
        let txt = ''
        for(let i = 0; i < model.count; i++) {
            txt = txt + model.get(i).txt + '\n'
        }
        return txt

    }

    id: root
    implicitHeight: view.implicitHeight
    implicitWidth: view.implicitWidth

    property Component delegate: TextEdit {
        font.pixelSize: 14
        width: view.width
        wrapMode: Text.WrapAnywhere
        readOnly: true
        textFormat: TextEdit.RichText
        text: txt
        color: palette.text
    }

    ListModel {
        id: model
    }

    ListView {
        id: view
        model: model
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

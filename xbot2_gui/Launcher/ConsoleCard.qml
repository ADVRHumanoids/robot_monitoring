import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Common

import "../Common"
import "ConsoleCard.js" as Logic

Item {

    // property alias defaultHeight: card.defaultHeight
    // property alias hidden: card.collapsed
    property alias name: consoleText.placeholderText
    // property alias processNames: combo.model

    property bool scrollOnOutput: true

    function appendText(text) {
        Logic.appendText(text,
                         consoleText,
                         scrollOnOutput)
    }

    function clearText() {
        consoleText.text = ''
    }

    id: root
    implicitHeight: consoleText.implicitHeight
    implicitWidth: consoleText.implicitWidth

    ScrollView {

        id: textScroll
        contentWidth: availableWidth
        anchors.fill: parent

        TextArea {

            id: consoleText

            color: "white"
            readOnly: true
            width: textScroll.availableWidth

            placeholderText: "Console output"
            wrapMode: TextEdit.Wrap

            textFormat: TextEdit.RichText

            font.pixelSize: 14
        }

    }

}

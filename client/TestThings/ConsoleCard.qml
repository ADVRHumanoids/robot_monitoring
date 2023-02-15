import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

import "ConsoleCard.js" as Logic

Item {

    property alias defaultHeight: card.defaultHeight
    property alias hidden: card.hidden
    property alias name: card.name
    property alias processNames: combo.model

    function appendText(text) {
        Logic.appendText(text,
                         consoleText,
                         scrollOnOutputCheck.checked)
    }

    id: root
    implicitHeight: card.implicitHeight
    implicitWidth: card.implicitWidth


    Card {

        id: card

        width: root.width
        height: root.height

        name: combo.currentText

        toolButtons: [
            ComboBox {
                id: combo
                visible: count > 0
            }

        ]

        frontItem: ScrollView {

            id: textScroll
            anchors.fill: parent
            contentHeight: consoleText.height

            TextArea {

                id: consoleText

                width: textScroll.contentWidth
                color: "white"
                readOnly: true

                placeholderText: "Console output"
                wrapMode: TextEdit.Wrap

                textFormat: TextEdit.RichText

                font.pixelSize: 14
            }

        }

        backItem: GridLayout {
            anchors.fill: parent
            columns: 2
            CheckBox {
                Layout.columnSpan: 2
                id: scrollOnOutputCheck
                text: 'Scroll on output'
                checked: true
            }
            Item {
                Layout.fillHeight: true
                Layout.columnSpan: 2
            }

        }
    }

}

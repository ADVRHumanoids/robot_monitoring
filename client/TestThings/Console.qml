import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

Rectangle {

    id: root
    radius: 4
    height: grid.height + 32
    color: Qt.lighter(Material.background)

    MaterialResponsiveGrid {

        id: grid

        width: parent.width

        CheckBox {

            id: scrollOnOutputCheck
            text: "Scroll on output"
            checked: true
            checkable: true
        }

        Button {

            id: clearBtn
            text: "Clear console"
            onReleased: {
                consoleText.clear()
            }
        }


        TextArea {

            property int columnSpan: grid.columns

            id: consoleText
            color: "white"
            readOnly: true

            placeholderText: "Console output"
            wrapMode: TextEdit.Wrap

            textFormat: TextEdit.RichText

            function addText(str) {
                append(str)
                if(scrollOnOutputCheck.checked) {
                    cursorPosition = length - 1
                }
            }

        }
    }
}

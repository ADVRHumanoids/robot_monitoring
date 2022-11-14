import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

Rectangle {

    property alias consoleText: consoleText

    id: root
    radius: 4
    implicitHeight: grid.implicitHeight + 32
    color: CommonProperties.colors.cardBackground

    MaterialResponsiveGrid {

        id: grid

        anchors.fill: parent

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

        ScrollView {
            id: textScroll
            property int columnSpan: grid.columns
            height: root.height * 0.8

            TextArea {

                width: textScroll.contentWidth

                id: consoleText
                color: "white"
                readOnly: true

                placeholderText: "Console output"
                wrapMode: TextEdit.Wrap

                textFormat: TextEdit.RichText

                font.pixelSize: 14

                function addText(str) {
                    append(str)
                    if(scrollOnOutputCheck.checked) {
                        cursorPosition = length - 1
                    }
                }
            }
        }
    }
}

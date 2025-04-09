import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Font
import "../Common"

Item {

    function appendText(procName, text) {

        var i = 0

        if(procName === 'launcher') {
            i = -1
        }
        else {
            i = processNames.indexOf(procName)
        }

        let color = colors[(i+1) % colors.length]

        text = `<font color="${color}">` + text + '</font>'

        consoleRepeater.itemAt(i+2).appendText(text)

        if(i >= 0 && !procCheckRepeater.itemAt(i).checked) {
            return
        }

        consoleRepeater.itemAt(0).appendText(text)
    }

    property list<string> processNames: ['proc1', 'proc2', 'proc3']
    property list<string> hiddenProcessNames: []


    //
    id: root

    Component.onCompleted: {

    }

    implicitWidth: card.implicitWidth

    implicitHeight: card.implicitHeight

    property list<string> colors: [
        '#4DADF7',
        '#22E3A4',
        '#D27CFF',
        '#FF9F45',
        '#33FFD0',
        '#82E0AA',
        '#B388FF',
        '#5EEAD4'
    ]

    Card1 {
        id: card
        collapsable: false
        anchors.fill: parent
        name: 'Console Output'

        toolButtons: [
            Button {
                text: 'Copy'
                onClicked: {
                    let txt = consoleRepeater.itemAt(consoleCombo.currentIndex).getText()
                    appData.copyToClipboard(txt)
                }
                onDoubleClicked: {
                    if(!CommonProperties.config.testing) {
                        return
                    }

                    for(let i = 0; i < 10000; i++) {
                        appendText('launcher', i+'Example text Example text Example text Example text Example text Example text Example text Example text Example text Example text Example text Example text Example text ')
                    }
                }
            },

            Button {
                text: 'Clear'
                onClicked: {
                    consoleRepeater.itemAt(consoleCombo.currentIndex).clearText()
                }
            },
            Item { width: 10 },
            ComboBox {
                id: consoleCombo
                model: ['all', 'launcher'].concat(root.processNames)
            }
        ]

        frontItem: StackLayout {
            width: parent.width
            height: root.height - 80
            currentIndex: consoleCombo.currentIndex
            Repeater {
                id: consoleRepeater
                model: ['all', 'launcher'].concat(root.processNames)
                ConsoleCard {
                    name: modelData
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    scrollOnOutput: scrollOnOutputCheck.checked
                }
            }
        }

        backItem: GridLayout {

            id: grid

            columns: 3

            CheckBox {
                id: scrollOnOutputCheck
                text: 'Scroll on output'
                checked: true
            }

            Button {
                text: 'Clear all consoles'
                onClicked: {
                    for(let i = 0; i < consoleRepeater.count; i++) {
                        consoleRepeater.itemAt(i).clearText()
                    }
                }
            }

            Item {
                Layout.fillWidth: true
            }

            Label {
                text: 'Select the processes to show in the global console'
                font.pixelSize: CommonProperties.font.h3
                Layout.columnSpan: Math.max(1, grid.columns)
            }

            Repeater {
                id: procCheckRepeater
                model: root.processNames
                CheckBox {
                    text: modelData
                    checked: root.hiddenProcessNames.indexOf(modelData) === -1
                }
            }
        }

    }

}

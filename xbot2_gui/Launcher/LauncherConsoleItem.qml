import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Font
import Common

Item {

    function appendText(procName: string, text: string, muted: bool) {

        let i = 0

        if(procName === 'launcher') {
            i = -1
        }
        else {
            i = processNames.indexOf(procName)
        }

        let color = colors[(i+1) % colors.length]

        text = `<b><font color="${color}">[${procName}] </font></b>` + text

        consoleRepeater.itemAt(i+2).appendText(text)

        if(!scrollOnOutputCheck.checked) {
            _showNewOutputVisible = true
        }

        if(i >= 0 && muted) {
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

    property bool _showNewOutputVisible: false

    implicitWidth: card.implicitWidth

    implicitHeight: card.implicitHeight

    property list<string> colors: [
        '#82AAFF',
        '#89DDFF',
        '#C792EA',
        '#A6E3A1',
        '#9CCFD8',
        '#B4BEFE',
        '#7DCFFF',
        '#A1EFD3'
    ]

    Card1 {
        id: card
        collapsable: false
        anchors.fill: parent
        name: 'Console Output'

        toolButtons: [

            SmallToolButton {
                text: MaterialSymbolNames.copy
                font.family: 'Material Symbols Outlined'
                font.variableAxes: {'opsz': 48}
                font.pixelSize: 16

                onClicked: {
                    let txt = consoleRepeater.itemAt(consoleCombo.currentIndex).getText()
                    appData.copyToClipboard(txt)
                }
            },

            SmallToolButton {
                text: MaterialSymbolNames.clean
                font.family: 'Material Symbols Outlined'
                font.variableAxes: {'opsz': 48}
                font.pixelSize: 16

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

            anchors.fill: parent
            currentIndex: consoleCombo.currentIndex

            Repeater {

                id: consoleRepeater

                model: ['all', 'launcher'].concat(root.processNames)

                Item {
                    required property int index
                    required property string modelData
                    // property color txtColor: root.colors[index % root.colors.length]

                    function appendText(txt) {
                        // console.log(root.colors[index % root.colors.length])
                        // console.log(txtColor)
                        listModel.append({'txt': txt}) // , 'txtColor': txtColor})
                        if(scrollOnOutputCheck.checked) {
                            consoleLoader.item?.scrollToEnd()
                        }
                    }

                    function clearText() {
                        listModel.clear()
                    }

                    function getText() {
                        let txt = ''
                        for(let i = 0; i < listModel.count; i++) {
                            txt = txt + listModel.get(i).txt + '\n'
                        }
                        return txt
                    }

                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    implicitHeight: consoleLoader.implicitHeight
                    implicitWidth: consoleLoader.implicitWidth

                    Loader {

                        id: consoleLoader

                        active: index === consoleCombo.currentIndex

                        anchors.fill: parent

                        sourceComponent: ConsoleCard {
                            id: cardInner
                            pixelSize: fontSizeSpin.value
                            onFlickStarted: scrollOnOutputCheck.checked = false

                            Column {
                                anchors {
                                    top: parent.top
                                    horizontalCenter: parent.horizontalCenter
                                    margins: 8
                                }
                                opacity: 0.7

                                Button {
                                    id: scrollOnOutputButton
                                    text: 'Show new output'
                                    onClicked: {
                                        cardInner.scrollToEnd()
                                        scrollOnOutputCheck.checked = true
                                        _showNewOutputVisible = false
                                    }
                                    visible: _showNewOutputVisible
                                }
                            }
                        }

                        onLoaded: {
                            item.listModel = listModel
                            if(scrollOnOutputCheck.checked) {
                                item.scrollToEnd()

                            }
                            console.log(`${modelData} loaded ${listModel.count} lines`)
                        }

                    }

                    ListModel {
                        id: listModel
                    }

                }
            }
        }

        backItem: GridLayout {

            id: grid
            width: parent.width
            columns: Math.ceil(width / 450)
            uniformCellWidths: true
            uniformCellHeights: true

            FramedControl {
                Layout.fillWidth: true
                Layout.fillHeight: true
                text: 'Scroll on output'
                subtext: 'Automatically scroll console to show new output. This is disabled on manual scroll.'
                CheckBox {
                    id: scrollOnOutputCheck
                    checked: true
                }

            }

            FramedControl {
                Layout.fillWidth: true
                Layout.fillHeight: true
                text: 'Clear all'
                subtext: 'Clear output for all processes (including main console)'
                Button {
                    text: 'Clear'
                    onClicked: {
                        for(let i = 0; i < consoleRepeater.count; i++) {
                            consoleRepeater.itemAt(i).clearText()
                        }
                    }
                }
            }

            FramedControl {
                Layout.fillWidth: true
                Layout.fillHeight: true
                text: 'Font size'
                subtext: 'Adjust font size; applies to all consoles'
                SpinBox {
                    id: fontSizeSpin
                    from: 1
                    to: 18
                    value: 12
                }
            }
        }

    }

    Settings {
        category: 'console'
        property alias fontSize: fontSizeSpin.value
    }

}

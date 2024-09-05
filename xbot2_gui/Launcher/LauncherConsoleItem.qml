import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

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

        let color = theme.baseColors[(i+1) % theme.baseColors.length]

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

    implicitWidth: card.implicitWidth

    implicitHeight: card.implicitHeight

    // https://doc.qt.io/qt-6/qml-color.html
    Item {
        id: theme
        property list<Item> baseColors: [
            Item {
                property color color: 'white'
            },
            Item {
                property color color: 'aquamarine'
            },
            Item {
                property color color: 'darkkhaki'
            },
            Item {
                property color color: 'mediumspringgreen'
            },
            Item {
                property color color: 'greenyellow'
            },
            Item {
                property color color: 'lightpink'
            },
            Item {
                property color color: 'lightseagreen'
            }

        ]
    }

    Card1 {
        id: card
        collapsable: false
        anchors.fill: parent
        name: 'Console Output'

        toolButtons: [
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


    // Item {
    //     property string textAggregated
    //     property var textMap
    // }

    // ColumnLayout {

    //     anchors.fill: parent

    //     SectionHeader {

    //         text: 'Console Output'

    //         Layout.fillWidth: true

    //         Button {
    //             text: 'C'
    //             onClicked: {
    //                 cfg.visible = !cfg.visible
    //             }
    //         }

    //         ComboBox {
    //             model: ['All']
    //         }

    //         CheckBox {
    //             text: 'Autoscroll'
    //             checked: true
    //             id: autoscrollCheck
    //         }

    //     }


    //     Item {

    //         id: cfg
    //         clip: true

    //         Layout.fillWidth: true
    //         Layout.fillHeight: true
    //         Layout.preferredHeight: implicitHeight
    //         implicitHeight: grid.implicitHeight
    //         implicitWidth: grid.implicitWidth

    //         GridLayout {
    //             id: grid
    //             anchors.fill: parent
    //             Repeater {
    //                 model: root.processNames
    //                 CheckBox {
    //                     text: modelData
    //                 }
    //             }
    //         }
    //     }





}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import "../Common"

Item {

    function appendText(procName, text) {
        let i = processNames.indexOf(procName)
        consoleRepeater.itemAt(i+1).appendText(text)
        if(!procCheckRepeater.itemAt(i).checked) {
            return
        }
        consoleRepeater.itemAt(0).appendText(text)
    }

    property list<string> processNames: ['proc1', 'proc2', 'proc3']


    //
    id: root

    implicitWidth: card.implicitWidth

    implicitHeight: card.implicitHeight

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
                model: ['All'].concat(root.processNames)
            }
        ]

        frontItem: StackLayout {
            width: parent.width
            height: root.height - 80
            currentIndex: consoleCombo.currentIndex
            Repeater {
                id: consoleRepeater
                model: ['All'].concat(root.processNames)
                ConsoleCard {
                    name: modelData
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    scrollOnOutput: true
                }
            }
        }

        backItem: GridLayout {

            id: grid

            columns: 3

            CheckBox {
                text: 'Scroll on output'
                checked: true
                Layout.columnSpan: Math.max(1, grid.columns)
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
                    checked: true
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

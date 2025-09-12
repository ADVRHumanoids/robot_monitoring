import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Main
import Common
import Font
import Menu

Item {

    id: root

    property ClientEndpoint client

    property list<string> requestedPages: []

    property bool isCurrentPage: false

    function pageSelected() {

        if(isCurrentPage) {
            console.log('app page selected')
            swipe.currentIndex = 0
        }


        console.log('requesting pages...')

        client.doRequestAsync('GET', '/requested_pages', '')
        .then((res) => {
                  root.requestedPages = res.requested_pages
                  root.requestedPages.push('Plot') // always add Plot page')
                  console.log(`got requested pages ${root.requestedPages}`)
              }, '')
    }

    Item {

        id: appModel

        PageItem {
            name: "Plot"
            page: "/qt/qml/LivePlot/Plot.qml"
            iconText: MaterialSymbolNames.tableChart
            iconFont: syms.font.family
            active: client.isConnected || mainWindow.dbg
        }

        PageItem {
            name: "Horizon"
            page: "/qt/qml/Horizon/Horizon.qml"
            iconText: MaterialSymbolNames.walker
            iconFont: syms.font.family
            active: client.robotConnected || mainWindow.dbg || true
            sizeFactor: 1.1
            show: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Parameters"
            page: "/qt/qml/Monitoring/Parameters.qml"
            iconText: MaterialSymbolNames.tune
            iconFont: syms.font.family
            active: true
            show: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Builder"
            page: "/qt/qml/TestThings/Linfa.qml"
            iconText: MaterialSymbolNames.tools
            iconFont: syms.font.family
            active: true
            show: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Linfa"
            page: "/qt/qml/TestThings/Linfa.qml"
            iconSource: '/Icons/icons/alberobotics100x100.png'
            active: true
            show: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Drill Task"
            page: "/qt/qml/Concert/Drilling.qml"
            iconText: MaterialSymbolNames.drill
            iconFont: syms.font.family
            active: true
            sizeFactor: 1.2
            show: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Conda"
            page: "/qt/qml/Concert/Conda.qml"
            iconText: MaterialSymbolNames.wrench
            iconFont: syms.font.family
            active: true
            sizeFactor: 1.1
            show: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Sanding"
            page: "/qt/qml/Concert/Sanding.qml"
            iconSource: '/Icons/icons/brick_wall_white.png'
            active: true
            sizeFactor: 1.
            show: requestedPages.indexOf(name) > -1
        }

        PageItem {
            name: "Transportation"
            page: "/qt/qml/Concert/Transportation.qml"
            iconText: MaterialSymbolNames.weight
            iconFont: syms.font.family
            active: true
            show: requestedPages.indexOf(name) > -1
            sizeFactor: 1.1
        }

        PageItem {
            name: "Ecat"
            page: "/qt/qml/Ecat/Ecat.qml"
            iconText: MaterialSymbolNames.wrench
            iconFont: syms.font.family
            active: true
            sizeFactor: 1.1
            show: requestedPages.indexOf(name) > -1
        }
    }


    SwipeView {

        id: swipe
        anchors.fill: parent
        interactive: false
        clip: true

        ScrollView {

            id: scroll

            contentWidth: availableWidth

            ColumnLayout {

                width: scroll.availableWidth

                RowLayout {

                    Layout.fillWidth: true

                    Label {
                        Layout.fillWidth: true
                        text: 'App Selection'
                        font.pixelSize: CommonProperties.font.h1
                        bottomPadding: 16
                        topPadding: 16
                    }

                    Button {
                        id: showDisabledBtn
                        text: 'Show disabled apps'
                        checkable: true
                    }

                }

                GridLayout {

                    id: grid

                    Layout.fillHeight: true
                    Layout.maximumWidth: Math.min(800, scroll.availableWidth)
                    Layout.alignment: Qt.AlignHCenter
                    columns: Math.ceil(width / 200)
                    uniformCellHeights: true
                    uniformCellWidths: true

                    Repeater {

                        model: appModel.children

                        Item {

                            implicitHeight: 100
                            implicitWidth: 100

                            required property int index
                            required property var modelData

                            Layout.fillHeight: true
                            Layout.fillWidth: true

                            visible: btn.enabled || showDisabledBtn.checked

                            // DebugRectangle {
                            //     target: btn
                            // }

                            NavButton {
                                id: btn
                                padding: 30
                                text: modelData.name
                                iconChar: modelData.iconText
                                checkedDisplayMode: AbstractButton.TextUnderIcon
                                uncheckedDisplayMode: AbstractButton.TextUnderIcon
                                font.pointSize: Qt.application.font.pointSize
                                sizeFactor: modelData.sizeFactor * 1.25
                                anchors.centerIn: parent
                                onClicked: {
                                    appStack.currentIndex = index
                                    swipe.currentIndex = 1
                                    Qt.callLater(() => appRepeater.itemAt(index).item.pageSelected())
                                }
                                checkable: false
                                enabled: root.requestedPages.indexOf(modelData.name) > -1 && modelData.active
                                background: Item {}
                            }

                            Rectangle {
                                color: Qt.alpha(root.palette.highlight, btn.containsMouse ? 0.3 : 0.1)
                                anchors.centerIn: btn
                                radius: 8
                                border.color: Qt.darker(color)
                                border.width: 2
                                height: btn.height + 30
                                width: height
                                z: -1
                                Behavior on color {

                                    ColorAnimation {}
                                }
                            }


                        }

                    }

                }

            }

        }

        StackLayout {

            id: appStack



            Repeater {

                id: appRepeater

                model: appModel.children

                Loader {

                    required property int index
                    required property var modelData

                    Layout.fillHeight: true
                    Layout.fillWidth: true

                    active: index === appStack.currentIndex && swipe.currentIndex === 1

                    onLoaded: {
                        active = true
                        item.pageSelected()
                    }

                    Component.onCompleted: {
                        setSource(modelData.page, {'client': client})
                    }

                }

            }

        }

    }

}


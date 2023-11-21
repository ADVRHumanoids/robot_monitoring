import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Main
import Common
import "../LivePlot"
import "BarPlot"
import "SingleJointState"

import "Monitoring.js" as Logic

MultiColumnPage {

    property ClientEndpoint client

    columnItems: [leftItem, rightItem]

    id: root

    property Item leftItem: Item {

        objectName: 'Global'

        Layout.fillWidth: true
        Layout.fillHeight: true

        ScrollView {

            id: leftScroll

            anchors.fill: parent

            contentWidth: leftCol.width
            contentHeight: leftCol.height

            ColumnLayout {

                id: leftCol
                width: leftScroll.availableWidth

                Card {

                    id: livePlotCard
                    name: 'Live plot'
                    hidden: true

                    Layout.fillWidth: true
                    Layout.preferredHeight: hidden ?
                                                implicitHeight :
                                                root.height - 2*margins

                    toolButtons: [
                        SmallToolButton {
                            text: '\uf021'
                            font.family: CommonProperties.fontAwesome.solid.family
                            onClicked: livePlot.resetView()
                        }

                    ]

                    frontItem: Plotter {
                        id: livePlot
                        anchors.fill: parent
                        property real initialTime: -1.0
                    }


                    backItem: Item {
                        anchors.fill: parent
                        implicitHeight: heightSpin.implicitHeight
                        SpinBox {
                            id: heightSpin
                            from: 0
                            to: 1000
                            value: 300
                            editable: true
                            stepSize: 50
                        }
                    }
                }


                Card {
                    id: viewer3dCard
                    name: '3D Viewer'
                    hidden: true

                    Layout.fillWidth: true
                    Layout.preferredHeight: hidden ?
                                                implicitHeight :
                                                root.height - 2*margins

                    frontItem: RobotModelViewer {
                        id: robotViewer
                        anchors.fill: parent
                        client: root.client
                        backgroundColor: 'transparent'
                    }
                }


                Card {

                    name: barPlotCombo.currentText
                    configurable: false
                    width: parent.width

                    toolButtons: [
                        ComboBox {
                            id: barPlotCombo
                            model: barPlot.fieldNames
                            width: implicitWidth
                            wheelEnabled: true
                        }

                    ]

                    frontItem: BarPlotStack {
                        id: barPlot
                        width: parent.width
                        currentIndex: barPlotCombo.currentIndex

                        onJointClicked: jointName => {
                                            jointState.selectJoint(jointName)
                                        }
                    }
                }


                Item {
                    Layout.fillHeight: true
                }
            }
        }

    }

    property Item rightItem: ScrollView {

        objectName: 'Safety / Joint State'

        id: rightScroll
        Layout.fillHeight: true

        ColumnLayout {

            width: rightScroll.availableWidth

            JointDevice {
                id: jointDevice
                Layout.fillWidth: true
                hidden: true

                onSetSafetyState: (ok) => Logic.setSafetyState(ok)
                onSetFilterActive: (active) => Logic.setFilterActive(active)
                onSetFilterCutoff: (profile) => Logic.setFilterProfile(profile)
            }

            JointCommandCard {
                id: cmdCard
                Layout.fillWidth: true
                robotCmd: robotViewer.robotCmd
                client: root.client
                hidden: true

                onResetCmd: robotViewer.resetCmd()
            }

            Card {

                Layout.fillWidth: true
                configurable: false
                name: 'Joint <i>' + jointState.jointNames[jointState.currentIndex] + '</i>'

                frontItem: SingleJointStateStack {
                    id: jointState
                    anchors.fill: parent

                    onPlotAdded: (jName, fieldName) => {
                                     Logic.addJointStateSeries(livePlot, jName, fieldName)
                                     livePlotCard.hidden = false
                                 }
                }

            }

            Item {
                Layout.fillHeight: true
            }

        }

    }

    property alias jointDevice: jointDevice
    property alias barPlot: barPlot
    property alias livePlot: livePlot

    Component.onCompleted: {
        Logic.construct()
    }



}

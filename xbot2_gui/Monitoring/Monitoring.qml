import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Font
import Common
import Main
import Monitoring.BarPlot
import Monitoring.SingleJointState

import "Monitoring.js" as Logic

MultiPaneResponsiveLayout {

    property ClientEndpoint client
    property Item robotViewer: loader.item

    id: root

    onBeforeLayoutChange: loader.active = false
    onAfterLayoutChange: loader.active = true

    Item {

        id: leftRoot
        width: parent.width

        Column {

            width: parent.width
            spacing: 16

            JointDevice {
                id: jointDevice
                width: parent.width
                collapsed: true
                onSetSafetyState: Logic.setSafetyState(ok)
                onSetFilterActive: Logic.setFilterActive(ok)
                onSetFilterCutoff: Logic.setFilterProfile(profile)
            }

            ScrollView {

                property string iconText: 'Telemetry'
                property string iconChar: MaterialSymbolNames.analytics

                id: scroll1

                width: parent.width
                height: leftRoot.height - jointDevice.height - parent.spacing - parent.topPadding - parent.bottomPadding
                contentWidth: availableWidth


                Column {

                    width: scroll1.availableWidth
                    spacing: 16



                    Card1 {

                        property int columnSize: 2
                        width: parent.width
                        name: barPlotCombo.currentText
                        configurable: false

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
                                                jointCommand.selectJoint(jointName)
                                            }
                        }
                    }

                    Card1 {

                        property int columnSize: 1
                        Layout.minimumWidth: implicitWidth

                        width: parent.width
                        configurable: false
                        name: 'Joint <i>' + jointState.jointNames[jointState.currentIndex] + '</i>'

                        frontItem: SingleJointStateStack {
                            id: jointState
                            width: parent.width

                            onPlotAdded: (jName, fieldName) => {
                                             Logic.addJointStateSeries(livePlot, jName, fieldName)
                                             livePlotCard.hidden = false
                                         }

                        }

                    }

                }

            }

        }

    }


    ColumnLayout {

        property string iconText: 'Control'
        property string iconChar: MaterialSymbolNames.box3d

        anchors.fill: parent

        Loader {
            id: loader
            width: parent.width

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 200

            sourceComponent: Component {
                RobotModelViewer {
                    id: robotViewer
                    implicitHeight: 200
                    implicitWidth: 200

                    client: root.client
                    backgroundColor: 'transparent'
                }
            }

        }

        JointCommandCard {
            id: jointCommand
            Layout.fillWidth: true
            client: root.client
            robotCmd: loader.item.robotCmd
            onResetCmd: robotViewer.resetCmd()
        }

    }

    Connections {
        target: client
        function onJointStateReceived(js) {
            barPlot.setJointStateMessage(js)
            jointState.setJointStateMessage(js)
            robotViewer.updateRobotState(js,
                                         robotViewer.robotState,
                                         'linkPos')
        }
    }

}

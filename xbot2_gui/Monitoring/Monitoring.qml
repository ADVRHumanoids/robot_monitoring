import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Font
import Common
import Main
import Monitoring.BarPlot
import Monitoring.SingleJointState
import ViewerQuick3D

import "Monitoring.js" as Logic

MultiPaneResponsiveLayout {

    property ClientEndpoint client
    property Item robotViewer: loader.item
    enabled: client.robotConnected
    property bool isCurrentPage

    id: root
    property Item livePlot: CommonProperties.globalLivePlot
    // onBeforeLayoutChange: loader.active = false
    // onAfterLayoutChange: loader.active = true
    property real vbatt
    property real iload

    // LayoutClassHelper {
    //     id: layout
    //     targetWidth: root.width
    // }

    Item {

        property string iconText: 'Telemetry'
        property string iconChar: MaterialSymbolNames.barchart

        id: leftRoot
        width: parent.width

        Column {

            width: parent.width
            spacing: 16

            // safety, filters, and battery
            GridLayout {

                id: jointDeviceGrid

                width: parent.width

                rows: layout.compact ? -1 : 1
                columns: layout.compact ? 1 : -1

                rowSpacing: 8
                columnSpacing: 8

                JointDevice {
                    id: jointDevice
                    Layout.fillWidth: true
                    Layout.preferredWidth: 1
                    // Layout.preferredHeight: height
                    bannerHeight: batt.height
                    collapsed: true
                    onSetSafetyState: Logic.setSafetyState(ok)
                    onSetFilterActive: Logic.setFilterActive(ok)
                    onSetFilterCutoff: Logic.setFilterProfile(profile)
                }


                Control {
                    id: batt
                    Layout.fillWidth: true
                    Layout.preferredWidth: 1
                    Layout.alignment: Qt.AlignTop

                    padding: 10

                    background: Rectangle {
                        color: CommonProperties.colors.cardBackground
                        radius: CommonProperties.geom.cardRadius
                    }

                    contentItem: RowLayout {
                        spacing: 16
                        TextField {
                            Layout.fillWidth: true
                            placeholderText: 'vbatt'
                            text: root.vbatt.toFixed(1) + ' V'
                            font.pixelSize: CommonProperties.font.h3
                            readOnly: true
                        }
                        TextField {
                            Layout.fillWidth: true
                            placeholderText: 'iload'
                            text: root.iload.toFixed(1) + ' A'
                            font.pixelSize: CommonProperties.font.h3
                            readOnly: true
                        }
                    }
                }
            }

            ScrollView {

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
                                Layout.fillWidth: true
                                model: barPlot.fieldNames
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

                            onSelectedJointsChanged: {
                                jointCommand.ctrlJoints = selectedJoints
                                loader.item.selectedJoints = selectedJoints
                            }

                            enableMultipleSelection: jointCommand.enableMultipleSelection
                        }

                    }

                    Card1 {

                        property int columnSize: 1
                        Layout.minimumWidth: implicitWidth

                        width: parent.width
                        configurable: false
                        name: 'Joint <i>' + jointState.jointNames[jointState.currentIndex] + '</i>'

                        frontItem: SingleJointState1 {
                            id: jointState
                            width: parent.width

                            onPlotAdded: function(jName, fieldName) {
                                Logic.addJointStateSeries(livePlot, jName, fieldName)
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
            asynchronous: true
            active: true

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 200

            sourceComponent: RobotModelViewer {
                id: robotViewer
                client: root.client
                color: Qt.transparent

                onJointClicked: function(jointName) {
                    jointState.selectJoint(jointName)

                }

                onSelectedJointsChanged: {
                    barPlot.selectedJoints = selectedJoints
                    jointCommand.ctrlJoints = selectedJoints
                }

                enableMultipleSelection: jointCommand.enableMultipleSelection

            }

        }

        JointCommandCard {
            id: jointCommand
            collapsed: true
            Layout.fillWidth: true
            client: root.client
            robotCmd: loader.item.robotCmd
            onResetCmd: robotViewer.resetCmd()
            onCmdChanged: robotViewer.showRobotCmd = true
            enabled: jointDevice.jointActive
            // ctrlJoints: loader.item.selectedJoints
            onJointRemoved: {
                barPlot.selectedJoints = ctrlJoints
                loader.item.selectedJoints = ctrlJoints
            }
        }

        Gripper {

            collapsed: true
            Layout.fillWidth: true
            visible: gripperNames.length > 0

        }

    }

    Connections {

        target: client

        function onJointStateReceived(js) {
            Logic.jsCallback(js)
        }

        function onObjectReceived(obj) {
            Logic.objCallback(obj)
        }
    }

}

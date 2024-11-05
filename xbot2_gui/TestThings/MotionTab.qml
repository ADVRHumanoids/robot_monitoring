import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebView

import Common
import Main
import ExpandableBottomBar
import Font
import Menu
import Joy
import LivePlot

import "MotionTab.js" as Logic

import ViewerQuick3D as V


Control {

    property ClientEndpoint client

    property real initialTime: -1

    property string trjPluginState

    property alias robotViewer: viewerLoader.item


    topPadding: 16

    id: root

    contentItem: ColumnLayout {

        spacing: 16

        RowLayout {



            Frame {

                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.preferredWidth: 1

                padding: 16

                ColumnLayout {

                    anchors.fill: parent

                    RowLayout {

                        spacing: 24

                        Button {
                            text: 'Configuration'
                            onClicked: {
                                motorConfigPopup.refresh()
                                motorConfigPopup.open()
                            }
                        }

                        Button {
                            text: 'Connect'
                            onClicked: Logic.connect()
                            enabled: cfg.configured
                        }

                        TextArea {
                            id: statusText
                            placeholderText: 'Status'
                            readOnly: true
                            text: !cfg.configured ? 'Configuration missing' : 'Configured'
                            Layout.fillWidth: true
                        }

                    }

                    StackLayout {

                        id: stack

                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        Layout.columnSpan: 3
                        clip: true


                        Control {
                            Layout.fillHeight: true
                            Layout.fillWidth: true
                            contentItem: Button {
                                text: 'Start acquisition'
                                onClicked: Logic.startAcquisition()
                            }
                        }

                        Control {

                            contentItem: GridLayout {

                                columns: 1

                                TextArea {
                                    text: 'Trajectory 1/4 with duration = '
                                }

                                ProgressBar {
                                    Layout.fillWidth: true
                                }
                            }
                        }

                    }

                    RowLayout {

                        Item {
                            Layout.fillWidth: true
                        }

                        Button {
                            text: 'Next'
                            onClicked: stack.currentIndex = stack.currentIndex + 1
                        }

                        Button {
                            text: 'Cancel'
                            onClicked: stack.currentIndex = 0
                        }

                    }



                }

            }

            Loader {

                id: viewerLoader

                asynchronous: true

                active: true

                Layout.fillWidth: true

                Layout.preferredWidth: 1
                Layout.preferredHeight: 300

                sourceComponent: V.RobotModelViewer {
                    client: root.client
                }

            }

        }

        GridLayout {

            Layout.fillWidth: true
            Layout.fillHeight: true
            columns: 2

            Plotter {

                id: positionPlot
                Layout.fillWidth: true
                Layout.fillHeight: true

                plotterLegend: positionLegend

                interactive: false

                chartView.title: 'Position'
                chartView.titleColor: palette.text
                chartView.margins {
                    bottom: 6
                    left: 6
                    right: 6
                    top: 6
                }

                property var linkSeries
                property var motSeries
                property var refSeries

                PlotterLegend {
                    id: positionLegend
                    chart: parent.chartView
                    visible: true
                }

            }

            Plotter {

                id: velocityPlot
                Layout.fillWidth: true
                Layout.fillHeight: true

                plotterLegend: velocityLegend

                interactive: false

                chartView.title: 'Velocity'
                chartView.titleColor: palette.text
                chartView.margins {
                    bottom: 6
                    left: 6
                    right: 6
                    top: 6
                }

                property var linkSeries
                property var motSeries
                property var refSeries

                PlotterLegend {
                    id: velocityLegend
                    chart: parent.chartView
                    visible: true
                }

            }

            Plotter {

                id: torquePlot

                plotterLegend: torqueLegend
                Layout.fillWidth: true
                Layout.fillHeight: true

                interactive: false

                chartView.title: 'Torque'
                chartView.titleColor: palette.text
                chartView.margins {
                    bottom: 6
                    left: 6
                    right: 6
                    top: 6
                }

                property var linkSeries
                property var motSeries
                property var refSeries

                PlotterLegend {
                    id: torqueLegend
                    chart: parent.chartView
                    visible: true
                }

            }

            Plotter {

                id: frictionPlot
                Layout.fillWidth: true
                Layout.fillHeight: true

                plotterLegend: frictionLegend

                interactive: false

                chartView.title: 'Friction'
                chartView.titleColor: palette.text
                chartView.margins {
                    bottom: 6
                    left: 6
                    right: 6
                    top: 6
                }

                property var linkSeries
                property var motSeries
                property var refSeries

                PlotterLegend {
                    id: frictionLegend
                    chart: parent.chartView
                    visible: true
                }

            }

        }

    }

    Popup {

        id: motorConfigPopup

        function refresh() {
            cfg.refresh()
        }

        ConfigureMotorPopup {
            id: cfg
            anchors.fill: parent
            client: root.client
            onDone: motorConfigPopup.close()
        }

        anchors.centerIn: Overlay.overlay
        // width: Overlay.overlay.width * 0.8
        // height: Overlay.overlay.height * 0.8
        padding: 16

        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        clip: true
    }

    Component.onCompleted: Logic.construct()

    Connections {
        target: client

        function onJointStateReceived (msg) {
            Logic.jsCallback(msg)
        }

        function onObjectReceived (msg) {
            if(msg.type === 'plugin_stats') {
                root.trjPluginState = msg['trajectory'].state
            }
        }
    }

}


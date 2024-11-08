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
import QtQml.StateMachine as SM


Control {

    property ClientEndpoint client

    property real initialTime: -1

    property string trjPluginState

    property alias robotViewer: viewerLoader.item

    property string dateTime

    property real trjProgress


    id: root

    SM.StateMachine {
        id: sm
        running: true
        initialState: unconfigured

        SM.State {
            id: notConfigured
            SM.SignalTransition {
                targetState: notConnected
            }
        }

        SM.State {
            id: notConnected
        }


    }

    topPadding: 16

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
                    spacing: 24

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
                        clip: true
                        enabled: statusText.text === 'Connected'


                        Control {
                            Layout.fillHeight: true
                            Layout.fillWidth: true
                            contentItem: ColumnLayout {

                                Text {
                                    Layout.fillWidth: true
                                    color: palette.active.text
                                    text: 'Press <i>Start acquisition</i> to begin the data acquisition procedure, that will guide you through the execution of n=${numTrj} trajectories. <br/><br/>Select the <i>Test Run</i> switch to mark this run as not to be used for calibration.'
                                    wrapMode: Text.WordWrap
                                    font.pointSize: 12
                                    // readOnly: true

                                }

                                Button {
                                    Layout.alignment: Qt.AlignHCenter
                                    text: 'Start acquisition'
                                    onClicked: {
                                        root.trjProgress = -1.0
                                        root.dateTime = new Date().toLocaleString('en-US', {'second': 'numeric'});
                                        stack.currentIndex = 1
                                    }
                                }

                                Switch {
                                    id: testRunSwitch
                                    Layout.alignment: Qt.AlignHCenter
                                    text: 'Test Run'
                                }
                            }
                        }

                        Repeater {

                            model: cfg.selectedMotorProperties.trajectory.length

                            Control {

                                required property int index

                                property var trj: cfg.selectedMotorProperties.trajectory[index]

                                contentItem: GridLayout {

                                    columns: 2

                                    Label {
                                        text: `Trajectory #${index+1} (name "${trj.name}")`
                                        Layout.columnSpan: 2
                                        Layout.alignment: Qt.AlignHCenter
                                        font.pointSize: 12
                                        padding: 8
                                    }

                                    Label {
                                        text: 'Amplitude'
                                    }

                                    TextField {
                                        text: trj.amplitude
                                        readOnly: true
                                        enabled: false
                                    }

                                    Label {
                                        text: 'Omega min.'
                                    }

                                    TextField {
                                        text: trj.omega_min
                                        readOnly: true
                                        enabled: false
                                    }

                                    Label {
                                        text: 'Omega max.'
                                    }

                                    TextField {
                                        text: trj.omega_max
                                        readOnly: true
                                        enabled: false
                                    }

                                    Label {
                                        text: 'Locked output'
                                    }

                                    TextField {
                                        text: trj.locked_output
                                        readOnly: true
                                        enabled: false
                                    }

                                    ProgressBar {
                                        Layout.fillWidth: true
                                        Layout.columnSpan: 2
                                        indeterminate: root.trjProgress < 0
                                        value: root.trjProgress
                                    }

                                    Button {
                                        text: 'Start'
                                        onClicked: startDialog.open()

                                        Dialog {

                                            id: startDialog
                                            modal: true
                                            anchors.centerIn: Overlay.overlay
                                            standardButtons: Dialog.Ok | Dialog.Cancel

                                            Text {
                                                text: `Make sure that the load is ${trj.locked_output ? "LOCKED" : "UNLOCKED"}, then press OK to continue. The motor will start moving`
                                                color: palette.active.text
                                                font.pointSize: 14
                                            }

                                            onAccepted: {
                                                trj['date_time'] = appData.getDateTime()
                                                trj['test_run'] = testRunSwitch.checked
                                                Logic.startAcquisition(trj)
                                            }
                                        }
                                    }
                                }
                            }

                        }

                    }

                    RowLayout {

                        Layout.fillWidth: true

                        Item {
                            Layout.fillWidth: true
                        }

                        Button {
                            text: 'Next'
                            onClicked: {
                                root.trjProgress = -1.0
                                stack.currentIndex = stack.currentIndex + 1
                            }
                            enabled: root.trjProgress >= 1.0 && stack.currentIndex > 0
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
            if(msg.type === 'hhcm_calib') {
                root.trjProgress = msg.progress
            }
        }
    }

}


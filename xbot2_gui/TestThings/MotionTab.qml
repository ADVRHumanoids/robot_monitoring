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
import Launcher

import "MotionTab.js" as Logic

import ViewerQuick3D as V
import QtQml.StateMachine as SM


Control {

    property ClientEndpoint client

    property real initialTime: -1

    property string trjPluginStatus

    property string xbot2Status

    property string calibDataDir

    property alias robotViewer: viewerLoader.item

    property string dateTime

    property real trjProgress

    property int numTrj: motorProperties.trajectory.length

    property var motorProperties

    property string motorId: 'NONE'

    property string motorType: 'NONE'

    property real loadMass: -1

    property real loadRadius: -1

    signal disconnectCommand()

    signal connectCommand()

    signal stopped()

    signal connectionError()

    signal connected()

    signal acquisitionStarted()

    signal acquisitionCanceled()

    //
    id: root

    SM.StateMachine {
        id: sm
        running: true
        initialState: stateNotConnected


        // not connected
        SM.State {
            id: stateNotConnected
            onEntered: statusText.text = 'Not connected'
            initialState: state0

            SM.State {
                id: state0
            }

            // connection error
            SM.State {
                id: stateConnectionError
                property string reason: 'undefined'
                onEntered: statusText.text = 'Connection error: ' + reason
            }

            // stopping
            SM.State {
                id: stateDisconnecting
                onEntered: Logic.stop()

                SM.SignalTransition {
                    signal: root.stopped
                    targetState: stateNotConnected
                }
            }

            SM.SignalTransition {
                signal: root.connectCommand
                targetState: stateConnectionInProgress
            }
        }

        // connection in progress
        SM.State {
            id: stateConnectionInProgress
            initialState: stateStopping

            SM.State {
                id: stateStopping
                onEntered: Logic.stop()
                SM.SignalTransition {
                    signal: root.stopped
                    targetState: stateStarting
                }
            }

            SM.State {
                id: stateStarting
                onEntered: Logic.connect()
                SM.SignalTransition {
                    signal: root.connected
                    targetState: stateConnected
                }
            }

            SM.SignalTransition {
                signal: root.connectionError
                targetState: stateConnectionError
            }
        }

        // connected
        SM.State {
            id: stateConnected
            initialState: stateIdle
            onEntered: {
                viewerLoader.active = false
                viewerLoader.active = true
            }

            // connected but doing nothing
            SM.State {
                id: stateIdle
                onEntered: {
                    statusText.text = 'Connected'
                }

                SM.SignalTransition {
                    signal: root.acquisitionStarted
                    targetState: stateAcquisitionInProgress
                }

                SM.SignalTransition {
                    signal: connectBtn.clicked
                    targetState: stateConnected
                }
            }

            // connected, started acquisition procedure
            SM.State {
                id: stateAcquisitionInProgress
                initialState: stateAcquisitionIdle

                SM.SignalTransition {
                    signal: root.acquisitionCanceled
                    targetState: stateConnected
                }

                // trj not running
                SM.State {
                    id: stateAcquisitionIdle

                    onEntered: statusText.text = `Trajectory ${stack.currentIndex}/${stack.count - 2}: not running`

                    SM.SignalTransition {
                        signal: acquisitionCompletedBtn.clicked
                        targetState: stateConnected
                    }

                    SM.SignalTransition {
                        signal: trjPluginStatusChanged
                        guard: trjPluginStatus === 'Running'
                        targetState: stateAcquisitionRunning
                    }
                }

                // trj running
                SM.State {
                    id: stateAcquisitionRunning
                    onEntered: statusText.text = `Trajectory ${stack.currentIndex}/${stack.count - 2}: running`

                    SM.SignalTransition {
                        signal: trjPluginStatusChanged
                        guard: trjPluginStatus === 'Stopped'
                        targetState: stateAcquisitionIdle
                    }
                }
            }

            SM.SignalTransition {
                signal: root.xbot2StatusChanged
                guard: root.xbot2Status !== 'Running'
                targetState: stateConnectionError
                onTriggered: stateConnectionError.reason = 'xbot2 not running'
            }

            SM.SignalTransition {
                signal: root.disconnectCommand
                targetState: stateDisconnecting
            }
        }
    }

    topPadding: 16

    contentItem: RowLayout {

        Frame {

            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: 1

            padding: 16

            ColumnLayout {

                anchors.fill: parent
                spacing: 24


                RowLayout {

                    spacing: 40

                    Label {
                        font.pointSize: CommonProperties.font.h1
                        text: 'Data acquisition wizard'
                    }

                    TextArea {
                        id: statusText
                        placeholderText: 'Status'
                        readOnly: true
                        text: '--'
                        Layout.fillWidth: true
                    }

                    Button {
                        text: 'Disconnect'
                        visible: !stateNotConnected.active
                        onClicked: root.disconnectCommand()
                    }

                    Button {
                        text: 'Connect'
                        onClicked: connectDialog.open()
                        visible: stateNotConnected.active

                        Dialog {

                            id: connectDialog
                            modal: true
                            anchors.centerIn: Overlay.overlay
                            standardButtons: Dialog.Ok | Dialog.Cancel
                            padding: 12

                            GridLayout {

                                columns: 2
                                columnSpacing: 8
                                rowSpacing: 8

                                Text {
                                    Layout.columnSpan: 2
                                    text: `Confirm attached load properties, then press OK to start the motor.`
                                    color: palette.active.text
                                    font.pointSize: 12
                                    bottomPadding: 6
                                }

                                Label {
                                    text: 'Load Mass [kg]'
                                }

                                TextField {
                                    id: loadMassText
                                    placeholderText: 'Mass [kg]'
                                    text: `0`
                                    validator: DoubleValidator {
                                        bottom: 0
                                        top: 50
                                        notation: DoubleValidator.StandardNotation
                                    }
                                }

                                Label {
                                    text: 'Load Radius [cm]'
                                }

                                TextField {
                                    id: loadRadiusText
                                    placeholderText: 'Radius [cm]'
                                    text: `0`
                                    validator: DoubleValidator {
                                        bottom: 0
                                        top: 50
                                        notation: DoubleValidator.StandardNotation
                                    }
                                }

                            }

                            onAccepted: {
                                root.loadMass = parseFloat(loadMassText.text)
                                root.loadRadius = parseFloat(loadRadiusText.text)*0.01
                                root.connectCommand()
                                close()
                            }
                        }
                    }

                }

                RowLayout {

                    spacing: 16

                    Label {
                        text: 'Load Properties '
                    }

                    TextField {
                        placeholderText: 'Mass [kg]'
                        text: root.loadMass.toFixed(2)
                        readOnly: true
                    }

                    TextField {
                        placeholderText: ' Radius [cm]'
                        text: (root.loadRadius*100).toFixed(2)
                        readOnly: true
                    }

                    Label {
                        leftPadding: 16
                        text: 'Motor Properties '
                    }


                    TextArea {
                        placeholderText: 'Motor ID (Motor Type)'
                        Layout.fillWidth: true
                        readOnly: true
                        text: `${root.motorId} (type ${root.motorType})`
                    }


                    // TextArea {
                    //     placeholderText: 'Calib. Data Location'
                    //     readOnly: true
                    //     text: root.calibDataDir
                    //     id: calibDataLocText
                    //     visible: false
                    // }

                    // ToolButton {
                    //     text: 'Copy'
                    //     onClicked: {
                    //         calibDataLocText.selectAll()
                    //         calibDataLocText.copy()
                    //     }
                    // }

                }

                // RowLayout {

                //     spacing: 24

                //     // Button {
                //     //     text: 'Configure'
                //     //     onClicked: {
                //     //         motorConfigPopup.refresh()
                //     //         motorConfigPopup.open()
                //     //     }
                //     // }







                // }

                StackLayout {

                    id: stack

                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    clip: true
                    enabled: stateConnected.active

                    Control {
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        contentItem: ColumnLayout {

                            Text {
                                Layout.fillWidth: true
                                color: palette.text
                                text: `Press <i>Start acquisition</i> to begin the data acquisition procedure, that will guide you through the execution of n=${numTrj} trajectories. <br/><br/>Select the <i>Test Run</i> switch to mark this run as not to be used for calibration.`
                                wrapMode: Text.WordWrap
                                font.pointSize: 11
                                // readOnly: true

                            }

                            Button {
                                Layout.alignment: Qt.AlignHCenter
                                text: 'Start acquisition'
                                onClicked: {
                                    root.trjProgress = -1.0
                                    root.dateTime = appData.getDateTime()
                                    stack.currentIndex = 1
                                    root.acquisitionStarted()
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

                        model: motorProperties.trajectory.length

                        Control {

                            required property int index

                            property var trj: motorProperties.trajectory[index]

                            contentItem: GridLayout {

                                columns: 4

                                Label {
                                    text: `Trajectory #${index+1} (name "${trj.name}")`
                                    Layout.columnSpan: 4
                                    Layout.alignment: Qt.AlignHCenter
                                    font.pointSize: 11
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

                                Label {
                                    text: 'Progress '
                                }

                                ProgressBar {
                                    Layout.fillWidth: true
                                    Layout.columnSpan: 3
                                    indeterminate: root.trjProgress < 0
                                    value: root.trjProgress
                                }

                                Button {
                                    text: 'Start'
                                    onClicked: startDialog.open()
                                    visible: stateAcquisitionIdle.active

                                    Dialog {

                                        id: startDialog
                                        modal: true
                                        anchors.centerIn: Overlay.overlay
                                        standardButtons: Dialog.Ok | Dialog.Cancel

                                        Text {
                                            text: `Make sure that the load is ${trj.locked_output ? "LOCKED" : "UNLOCKED"}, then press OK to continue. The motor will start moving`
                                            color: palette.active.text
                                            font.pointSize: 12
                                        }

                                        onAccepted: {
                                            trj['date_time'] = root.dateTime
                                            trj['test_run'] = testRunSwitch.checked
                                            root.trjProgress = -1
                                            Logic.startAcquisition(trj)
                                        }
                                    }
                                }

                                Button {
                                    text: 'Stop'
                                    onClicked: Logic.stopTrajectory()
                                    visible: stateAcquisitionRunning.active
                                }
                            }
                        }
                    }

                    Control {
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        padding: 0
                        contentItem: ColumnLayout {

                            Text {
                                Layout.fillWidth: true
                                text: 'Data acquisition completed. Press "Calibrate" to run a simple calibration.'
                                font.pointSize: 11
                                color: palette.text
                                wrapMode: Text.WordWrap
                            }

                            RowLayout {

                                Button {
                                    id: acquisitionCompletedBtn
                                    text: 'Calibrate'
                                    onClicked: Logic.calibrate()
                                }

                                Button {
                                    text: 'Upload'
                                    onClicked: Logic.upload()
                                }

                            }

                            ScrollView {

                                Layout.fillHeight: true
                                Layout.fillWidth: true
                                id: textScroll
                                TextArea {
                                    id: calibOutputText
                                    placeholderText: 'Console output'
                                    text: ''
                                    readOnly: true
                                    wrapMode: Text.WordWrap
                                }

                            }
                        }

                    }
                }

                RowLayout {

                    enabled: stack.enabled

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
                        enabled: stack.currentIndex < stack.count - 1 &&
                                 (testRunSwitch.checked || (root.trjProgress >= 1.0 && stack.currentIndex > 0))
                    }

                    Button {
                        text: stack.currentIndex === stack.count - 1 ? 'Finish' : 'Cancel'
                        onClicked: {
                            root.acquisitionCanceled()
                            stack.currentIndex = 0
                        }
                    }
                }
            }
        }

        Loader {

            id: viewerLoader

            asynchronous: true

            active: true

            visible: false

            Layout.fillWidth: true

            Layout.preferredWidth: 1
            Layout.preferredHeight: 300

            // sourceComponent: V.RobotModelViewer {
            //     client: root.client
            // }

        }



        GridLayout {

            id: plotGrid
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 1
            columns: 1
            readonly property int visibleMaskAll: 15
            property int visibleMask: visibleMaskAll

            RowLayout {
                spacing: 16
                Layout.alignment: Qt.AlignHCenter
                Button {
                    text: 'Reset View'
                    onClicked: {
                        positionPlot.resetView()
                        velocityPlot.resetView()
                        torquePlot.resetView()
                        frictionPlot.resetView()
                        frictionPlot.setXRange(-10, 10)
                    }
                }
                GroupBox {
                    visible: false
                    title: 'Time range [s]'
                    SpinBox {
                        id: timeSpanSpin
                        from: 1
                        to: 100
                        value: 10
                        editable: true
                    }

                }
            }

            MotionTabPlot {
                id: positionPlot
                readonly property int mask: 1
                title: 'Position'
                Layout.fillWidth: true
                Layout.fillHeight: true
                visible: plotGrid.visibleMask & mask
                timeSpan: timeSpanSpin.value
                onActivated: {
                    plotGrid.visibleMask =
                            plotGrid.visibleMask === plotGrid.visibleMaskAll ?
                                mask :
                                plotGrid.visibleMaskAll
                }
            }

            MotionTabPlot {
                id: velocityPlot
                readonly property int mask: 2
                title: 'Velocity'
                Layout.fillWidth: true
                Layout.fillHeight: true
                visible: plotGrid.visibleMask & mask
                timeSpan: timeSpanSpin.value
                onActivated: {
                    plotGrid.visibleMask =
                            plotGrid.visibleMask === plotGrid.visibleMaskAll ?
                                mask :
                                plotGrid.visibleMaskAll
                }
            }

            MotionTabPlot {
                id: torquePlot
                readonly property int mask: 4
                title: 'Torque'
                Layout.fillWidth: true
                Layout.fillHeight: true
                visible: plotGrid.visibleMask & mask
                timeSpan: timeSpanSpin.value
                onActivated: {
                    plotGrid.visibleMask =
                            plotGrid.visibleMask === plotGrid.visibleMaskAll ?
                                mask :
                                plotGrid.visibleMaskAll
                }
            }

            MotionTabPlot {
                id: frictionPlot
                readonly property int mask: 8
                title: 'Friction'
                Layout.fillWidth: true
                Layout.fillHeight: true
                xLabel: 'Velocity [rad/s]'
                visible: plotGrid.visibleMask & mask
                onActivated: {
                    plotGrid.visibleMask =
                            plotGrid.visibleMask === plotGrid.visibleMaskAll ?
                                mask :
                                plotGrid.visibleMaskAll
                }

                Component.onCompleted: setXRange(-10, 10)
            }

        }

    }

    // Popup {

    //     id: motorConfigPopup

    //     function refresh() {
    //         cfg.refresh()
    //     }

    //     ConfigureMotorPopup {
    //         id: cfg
    //         anchors.fill: parent
    //         client: root.client
    //         onDone: motorConfigPopup.close()
    //     }

    //     anchors.centerIn: Overlay.overlay
    //     // width: Overlay.overlay.width * 0.8
    //     // height: Overlay.overlay.height * 0.8
    //     padding: 16

    //     modal: true
    //     focus: true
    //     closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

    //     clip: true
    // }

    Component.onCompleted: Logic.construct()

    Connections {
        target: client

        function onJointStateReceived (msg) {
            Logic.jsCallback(msg)
        }

        function onPluginStatMessageReceived (msg) {
            trjPluginStatus = msg.trajectory.state
        }

        function onProcMessageReceived(msg) {
            if(msg.content === 'status' && msg.name === 'xbot2') {
                xbot2Status = msg.status
            }
            else if(msg.content === 'output' && msg.name === 'onedrive') {
                calibOutputText.text += msg.stdout
                calibOutputText.text += '\n'
            }
        }

        function onObjectReceived (msg) {
            if(msg.type === 'hhcm_calib') {
                root.trjProgress = msg.progress
            }
        }
    }

}


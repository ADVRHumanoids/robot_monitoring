import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick3D
import QtQuick3D.Helpers

import Main
import Common
import "/qt/qml/Main/sharedData.js" as SharedData
import "RobotModelViewer.js" as Logic

Rectangle {

    color: Qt.rgba(0.8, 0.8, 0.8, 1)

    property ClientEndpoint client
    property alias robotState: robotState
    property alias robotCmd: robotCmd
    property alias showRobotCmd: showCmdChk.checked

    signal jointClicked(string jointName)

    property alias selectedJoints: robotState.selectedJoints
    property bool enableMultipleSelection: false

    function updateRobotState(js, robot, fieldName) {
        Logic.updateViewerState(js, robot, fieldName)
    }

    function resetCmd() {
        Logic.updateViewerState(SharedData.latestJointState,
                                robotCmd,
                                'posRef')
    }

    function resetView() {
        originNode.resetView()
    }

    //
    id: root

    Control {

        z: 10

        padding: 8

        contentItem: GridLayout {
            columns: 1
            CheckBox {
                id: showAxesChk
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                // Layout.columnSpan: 2
                text: 'Show axes'
            }
            CheckBox {
                id: showCmdChk
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                // Layout.columnSpan: 2
                text: 'Show command robot'
            }
            Button {
                text: 'Reset view'
                onClicked: root.resetView()
                Layout.fillWidth: true
            }
        }

    }

    // The root scene
    Node {

        id: standAloneScene

        Node {

            id: originNode

            PerspectiveCamera {
                id: cameraPerspectiveTwo
                z: 200
                clipNear: 1
            }

            x: 50
            y: 100
            z: 50
            eulerRotation.y: 40
            eulerRotation.x: -40

            function resetView() {
                x = 50
                y = 100
                z = 50
                eulerRotation.y = 40
                eulerRotation.x = -40
                cameraPerspectiveTwo.z = 200
            }

        }

        Axes3D {
            visible: showAxesChk.checked
        }

        Node {

            id: modelScene

            DirectionalLight {
                ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
                brightness: 1
                eulerRotation.x: -25
            }

            DirectionalLight {
                ambientColor: Qt.rgba(-0.5, -0.5, 0.5, 1.0)
                brightness: 1
                eulerRotation.x: 25
            }

            RobotModelNode {
                id: robotState
                client: root.client
                eulerRotation.x: -90
                y: 75
                opacity: showAxesChk.checked ? 0.9 : 1
                color: 'green'
                axesVisible: showAxesChk.checked
            }

            RobotModelNode {
                id: robotCmd
                client: root.client
                eulerRotation.x: -90
                y: 75
                opacity: 0.5
                visible: showCmdChk.checked
            }

        }

    }

    View3D {

        anchors.fill: parent
        id: view3d
        importScene: standAloneScene
        camera: cameraPerspectiveTwo

        environment: SceneEnvironment {
                 backgroundMode: SceneEnvironment.Color
                 clearColor: palette.active.window
                 InfiniteGrid {
                     gridInterval: 30
                 }
             }

        OrbitCameraController {
            camera: cameraPerspectiveTwo
            origin: originNode
            anchors.fill: parent
        }

        MouseArea {
            anchors.fill: parent
            // property var lastPicked: undefined
            onClicked: function(mouse) {
                // try {
                //     lastPicked.isPicked = false
                // }
                // catch(err) {}

                var result = view3d.pick(mouse.x, mouse.y);
                var pickedObject = result.objectHit;
                // pickedObject.isPicked = !pickedObject.isPicked;
                console.log(pickedObject.parentJointName)
                root.jointClicked(pickedObject.parentJointName)

                if(pickedObject.isSelected) {
                    root.selectedJoints = root.selectedJoints.filter(item => item !== pickedObject.parentJointName)
                }
                else if(enableMultipleSelection) {
                    root.selectedJoints.push(pickedObject.parentJointName)
                }
                else {
                    root.selectedJoints = [pickedObject.parentJointName]
                }

            }
        }
    }
}

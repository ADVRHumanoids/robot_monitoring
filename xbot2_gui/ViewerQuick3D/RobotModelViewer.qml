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

    function updateRobotState(js, robot, fieldName) {
        Logic.updateViewerState(js, robot, fieldName)
    }

    function resetCmd() {
        Logic.updateViewerState(SharedData.latestJointState,
                                robotCmd,
                                'posRef')
    }

    //
    id: root

    Control {

        z: 10

        contentItem: GridLayout {
            columns: 2
            CheckBox {
                id: showAxesChk
                Layout.fillWidth: true
                Layout.columnSpan: 2
                text: 'Show axes'
            }
            CheckBox {
                id: showCmdChk
                Layout.fillWidth: true
                Layout.columnSpan: 2
                text: 'Show command robot'
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

        }

        Axes3D {
            visible: showAxesChk.checked
        }

        Node {

            id: modelScene

            DirectionalLight {
                ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
                brightness: 1.0
                eulerRotation.x: -25
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
                 clearColor: palette.active.base
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
            onClicked: (mouse) => {
                var result = view3d.pick(mouse.x, mouse.y);
                var pickedObject = result.objectHit;
                pickedObject.isPicked = !pickedObject.isPicked;
            }
        }
    }
}

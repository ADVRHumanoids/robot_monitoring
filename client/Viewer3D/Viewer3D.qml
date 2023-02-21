import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import ".."

Item {

    id: root

    property ClientEndpoint client

    RowLayout {
        id: btnRow
        width: parent.width

        Button {
            text: camTimer.running ? 'Stop' : 'Start'
            onClicked: camTimer.running ? camTimer.stop() : camTimer.start()
        }

        Button {
            text: 'Update TF'
            onClicked: robot.updateTf()
        }
    }

    Scene3D {
        id: scene3d

        anchors {
            top: btnRow.bottom
            bottom: parent.bottom
            left: parent.left
            right: parent.right
        }

        anchors.margins: 10
        focus: true
        aspects: ["input", "logic"]
        cameraAspectRatioMode: Scene3D.AutomaticAspectRatio
        entity: sceneRoot
    }

    Entity {
        id: sceneRoot

        Camera {
            id: camera
            projectionType: CameraLens.PerspectiveProjection
            fieldOfView: 70
            aspectRatio: 16/9
            nearPlane : 0.1
            farPlane : 10.0
            position: Qt.vector3d( 2.0, 0.0, 0.0 )
            upVector: Qt.vector3d( 0.0, 0.0, 1.0 )
            viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )

            Timer {
                id: camTimer
                repeat: true
                interval: 8
                onTriggered: {
                    camera.tiltAboutViewCenter(1)
                }
            }
        }

        CustomOrbitalController {
            camera: camera
        }

        components: [
            RenderSettings {
                activeFrameGraph: ForwardRenderer {
                    clearColor: Qt.rgba(0, 0.5, 1, 1)
                    camera: camera
                    showDebugOverlay: true
                }
            },
            // Event Source will be set by the Qt3DQuickWindow
            InputSettings { }
        ]

        SunLight {
            direction: Qt.vector3d(0, 1, -1)
            intensity: 0.3
            translation: Qt.vector3d(-3, -3, 3)
        }

        SunLight {
            direction: Qt.vector3d(0, -1, -1)
            intensity: 0.3
            translation: Qt.vector3d(-3, 3, 3)
        }

        RobotModel {
            id: robot
            client: root.client
            color: 'green'
            alpha: 1
        }


    }

    Component.onCompleted: {
        robot.updateModel()
    }

}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D



Item {

    property alias sceneVisible: scene3d.visible
    property color backgroundColor: 'transparent'
    property alias nodes: sceneRoot.childNodes

    // private
    id: root
    property bool _robotCmdFinalized: false
    property bool _robotStateFinalized: false

    Scene3D {
        id: scene3d
        width: Math.max(1, parent.width)
        height: Math.max(1, parent.height)
        focus: true
        aspects: ["input", "logic"]
        cameraAspectRatioMode: Scene3D.AutomaticAspectRatio
        entity: sceneRoot
    }

    CameraController {
        camera: camera
        anchors.fill: scene3d
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
        }

//        CustomOrbitalController {
//            camera: camera
//        }

        components: [
            RenderSettings {
                id: renderSettings
                activeFrameGraph: ForwardRenderer {
                    clearColor: root.backgroundColor
                    camera: camera
                }
            },
            // Event Source will be set by the Qt3DQuickWindow
            InputSettings { }
        ]

        SunLight {
            direction: Qt.vector3d(0, 1, -1)
            intensity: 0.5
            translation: Qt.vector3d(-30, -30, 30)
        }

        SunLight {
            direction: Qt.vector3d(0, -1, -1)
            intensity: 0.5
            translation: Qt.vector3d(-30, 30, 30)
        }

    }
}

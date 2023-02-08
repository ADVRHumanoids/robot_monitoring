import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D

import QtQuick

import ".."

Item {

    property ClientEndpoint client: undefined

    Scene3D {
        id: scene3d
        anchors.fill: parent
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
            fieldOfView: 45
            aspectRatio: 16/9
            nearPlane : 0.1
            farPlane : 10.0
            position: Qt.vector3d( 0.0, 0.0, -1.0 )
            upVector: Qt.vector3d( 0.0, 0.0, 1.0 )
            viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )
        }

        OrbitCameraController {
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

        NodeInstantiator {

            id: visualRepeater

            delegate: VisualEntity {
                source: "http://" + client.hostname + ":" + client.port + "/visual/get_mesh/" + encodeURIComponent(modelData.uri)
                Component.onCompleted: {
                    console.log(source)
                }
            }
        }
    }

    Component.onCompleted: {
        let loadMeshes = function(link_to_uri) {
            let model = []
            for (const [key, value] of Object.entries(link_to_uri)) {
                console.log(`${key}: ${value}`);
                let obj = {}
                obj.linkName = key
                obj.uri = value
                model.push(obj)
            }
            visualRepeater.model = model
        }
        client.doRequest('GET', '/visual/get_mesh_entities', '', loadMeshes)
    }

}

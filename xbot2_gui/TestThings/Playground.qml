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

import QtQuick3D
import QtQuick3D.Helpers

Item {

    property ClientEndpoint client

    //
    id: root

    SpinBox {
        id: camZSpin
        from: 0
        to: 1000
        value: 200
    }

    // The root scene
    Node {
        id: standAloneScene

        DirectionalLight {
            ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
            brightness: 1.0
            eulerRotation.x: -25
        }

        Model {
            source: "#Cube"
            x: 50
            scale: Qt.vector3d(1, 0.05, 0.05)
            materials: [
                DefaultMaterial {
                    diffuseColor: Qt.rgba(0.8, 0.0, 0.0, 1.0)
                }
            ]
        }

        Model {
            source: "#Cube"
            y: 50
            scale: Qt.vector3d(1, 0.05, 0.05)
            eulerRotation.z: 90
            materials: [
                DefaultMaterial {
                    diffuseColor: Qt.rgba(0.0, 0.8, 0.0, 1.0)
                }
            ]
        }

        Model {
            source: "#Cube"
            z: 50
            scale: Qt.vector3d(1, 0.05, 0.05)
            eulerRotation.y: -90
            materials: [
                DefaultMaterial {
                    diffuseColor: Qt.rgba(0., 0.0, 0.8, 1.0)
                }
            ]
        }

        Model {
            source: "#Sphere"
            scale: Qt.vector3d(0.1, 0.1, 0.1)
            materials: [
                DefaultMaterial {
                    diffuseColor: Qt.rgba(0.0, 0.8, 0.0, 0.2)
                }
            ]
        }

        RobotModelNode {
            client: root.client
        }

        // CustomMesh {
        //     //            name: modelData.linkName
        //     //            type: modelData.type
        //     meshUri: 'package://kyon_urdf/meshes/collision/mesh_pelvis.stl'
        //     // meshUri: '#Cube'
        //     // scale: Qt.vector3d(0.001, 0.001, 0.001)

        //     //            cylinderLength: modelData.length || 0
        //     //            cylinderRadius: modelData.radius || 0
        //     // scale: modelData.scale
        //     // localPosition: modelData.origin_xyz
        //     // localRotation: modelData.origin_rot
        //     // color: root.color
        //     // alpha: root.alpha
        //     // visible: root.visible
        //     client: root.client
        // }

        // Stationary perspective camera
        PerspectiveCamera {
            id: cameraPerspectiveTwo
            position: Qt.vector3d(200, 0, 0)
            clipNear: 1

            Component.onCompleted: lookAt(Qt.vector3d(0, 0, 0))
        }
    }

    View3D {
        anchors.fill: parent
        id: view3d
        importScene: standAloneScene
        camera: cameraPerspectiveTwo

        OrbitCameraController {
            camera: cameraPerspectiveTwo
            origin: standAloneScene
            anchors.fill: parent
        }
    }
}


import QtQuick
import QtQuick.Controls
import Qt3D.Core
import QtQuick.Window
import Qt3D.Extras
import Qt3D.Render
import Qt3D.Input
import QtQuick3D
import QtQuick3D.Helpers
import QtQuick3D.AssetUtils

import ".."

Item {

    property ClientEndpoint client: undefined

    Node {
        id: standAloneScene
        //! [rootnode]

        DirectionalLight {
            ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
            brightness: 1.0
            eulerRotation.x: -25
        }

        Model {
            source: "#Cube"
            y: -104
            scale: Qt.vector3d(3, 3, 0.1)
            eulerRotation.x: -90
            materials: [
                DefaultMaterial {
                    diffuseColor: Qt.rgba(0.8, 0.8, 0.8, 1.0)
                }
            ]
        }

        Mesh {
            source: Qt.resolvedUrl("https://code.qt.io/cgit/qt/qtquick3d.git/tree/examples/quick3d/view3d/teapot.mesh")
            onStatusChanged: {
                console.log('mesh status ', status)
            }
        }

        RuntimeLoader {

        }

//        Model {
////            source: Qt.resolvedUrl('file:///home/arturo/code/robots_ws/src/iit-centauro-ros-pkg/centauro_urdf/meshes/meshes/node.mesh')
////            source: "http://" + client.hostname + ":" + client.port + "/resources/pelvis.mesh"
//            source: Qt.resolvedUrl("https://code.qt.io/cgit/qt/qtquick3d.git/tree/examples/quick3d/view3d/teapot.mesh")
//            materials: [
//                PrincipledMaterial {
//                    baseColor: "#41cd52"
//                    metalness: 0.0
//                    roughness: 0.1
//                    opacity: 1.0
//                }
//            ]

//            PropertyAnimation on eulerRotation.y {
//                loops: Animation.Infinite
//                duration: 5000
//                to: 0
//                from: -360
//            }

//            Component.onCompleted: {
//                console.log(Qt.resolvedUrl(source))
//            }
//        }

        //! [cameras start]
        // The predefined cameras. They have to be part of the scene, i.e. inside the root node.
        // Animated perspective camera
        Node {
            PerspectiveCamera {
                id: cameraPerspectiveOne
                z: 600
            }

            PropertyAnimation on eulerRotation.x {
                loops: Animation.Infinite
                duration: 5000
                to: -360
                from: 0
            }
        }

        // Stationary perspective camera
        PerspectiveCamera {
            id: cameraPerspectiveTwo
            z: 600
        }
        //! [cameras start]

        // Second animated perspective camera
        Node {
            PerspectiveCamera {
                id: cameraPerspectiveThree
                x: 500
                eulerRotation.y: 90
            }
            PropertyAnimation on eulerRotation.y {
                loops: Animation.Infinite
                duration: 5000
                to: 0
                from: -360
            }
        }

        // Stationary orthographic camera viewing from the top
        OrthographicCamera {
            id: cameraOrthographicTop
            y: 600
            eulerRotation.x: -90
        }

        // Stationary orthographic camera viewing from the front
        OrthographicCamera {
            id: cameraOrthographicFront
            z: 600
        }

        //! [cameras end]
        // Stationary orthographic camera viewing from left
        OrthographicCamera {
            id: cameraOrthographicLeft
            x: -600
            eulerRotation.y: -90
        }
    }
    //! [cameras end]


    View3D {
        id: topRightView
        anchors.fill: parent
        camera: cameraPerspectiveOne
        importScene: standAloneScene
        renderMode: View3D.Underlay

        environment: SceneEnvironment {
            clearColor: "white"
            backgroundMode: SceneEnvironment.Color
        }
    }
}

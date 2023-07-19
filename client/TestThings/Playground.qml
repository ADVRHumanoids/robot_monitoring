import QtQuick
import QtQuick.Controls
import QtCharts

import ".."
import xbot2_gui.RobotModel
import xbot2_gui.MeshGeometry


import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers



Item {

    id: root
    property ClientEndpoint client


    View3D {

        id: v3d
        anchors.fill: parent
        camera: camera

        environment: SceneEnvironment {
            clearColor: "blue"
            antialiasingMode: SceneEnvironment.MSAA
//            backgroundMode: SceneEnvironment.Color

        }

        Node {

            id: originNode
            position: Qt.vector3d(2.0, 0, 0.0)
            eulerRotation: Qt.vector3d(90, 0, 0)

            PerspectiveCamera {
                id: camera

                clipNear: 0.05
                clipFar: 2
                fieldOfView: 90
            }

        }

        OrbitCameraController {
            anchors.fill: parent
            origin: originNode
            camera: camera
        }

        DirectionalLight {
            position: Qt.vector3d(-5, 5, -1)
            color: Qt.rgba(0.4, 0.2, 0.6, 1.0)
            ambientColor: Qt.rgba(0.1, 0.1, 0.1, 1.0)
        }

        PointLight {
            position: Qt.vector3d(0, 2, 2)
            color: Qt.rgba(1, 1, 1, 1.0)
            ambientColor: Qt.rgba(0.2, 0.2, 0.2, 1.0)
        }

        PointLight {
            position: Qt.vector3d(0, -2, 2)
            color: Qt.rgba(1, 1, 1, 1.0)
            ambientColor: Qt.rgba(0.2, 0.2, 0.2, 1.0)
        }

        RobotModelNode {
            client: root.client
        }

        Model {
            scale: Qt.vector3d(10, 10, 10)
            geometry: GridGeometry {
                horizontalLines: 100
                verticalLines: 100
            }
            materials: [ DefaultMaterial { } ]
        }


    }

    Component.onCompleted: {
        camera.lookAt(Qt.vector3d(0, 0, 0))
    }

}


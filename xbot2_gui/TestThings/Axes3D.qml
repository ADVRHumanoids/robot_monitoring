import QtQuick
import QtQuick3D

Node {



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

}

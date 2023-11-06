import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import xbot2_gui.MeshGeometry
import ".."

Node {

    property ClientEndpoint client

    property string meshUri

    property alias visible: model.visible
    property alias color: material.diffuseColor
    property alias alpha: material.opacity

    property alias scale: model.scale
    property alias localPosition: model.position
    property alias localRotation: model.rotation


    // private
    id: root
    property Geometry modelGeometry: MeshGeometry {
        id: geom
    }

    Model {
        id: model
        visible: true
        scale: Qt.vector3d(0.001, 0.001, 0.001)
        materials: [
            DefaultMaterial {
                id: material
            }
        ]

    }



    Component.onCompleted: {

        if(meshUri[0] === '#') {

            model.source = meshUri

        }
        else {

            let callback = (msg) => {
                console.log(`recv mesh ${meshUri}`)
                geom.meshFile = msg
                model.geometry = geom
            }

            let meshFilename = `/visual/get_mesh/${encodeURIComponent(meshUri)}`

            client.doRequestRaw('GET',
                                meshFilename,
                                '',
                                callback)
        }


    }

}

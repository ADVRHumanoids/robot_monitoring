import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import ViewerQuick3D
import Main
import Viewer3D as V

Node {

    property ClientEndpoint client

    property string meshUri

    property alias visible: model.visible
    property alias axesVisible: axes.visible
    property color color: Qt.rgba(0.8, 0.8, 0.8, 1.0)
    property alias alpha: material.opacity

    property alias scale: model.scale
    property alias localPosition: model.position
    property alias localRotation: model.rotation

    property real cylinderRadius
    property real cylinderLength

    // private
    id: root

    property Geometry modelGeometry: MeshGeometry {
        id: geom
    }

    Node {

        id: wrapperNode

        Model {
            id: model
            visible: true
            pickable: true
            property bool isPicked: false
            materials: [
                DefaultMaterial {
                    id: material
                    diffuseColor: model.isPicked ?
                                      Qt.lighter(root.color) :
                                      root.color
                }
            ]
        }

        Axes3D {
            id: axes
            scale: Qt.vector3d(0.1, 0.1, 0.1)
        }

    }


    V.CachedVisual {

        id: cachedVisual

        onMeshReady: {
            geom.setUrl(cachedVisual.file)
            model.geometry = geom
        }
    }

    Component.onCompleted: {

        if(meshUri[0] === '#') {

            let meshMap = Object()
            meshMap['#CYLINDER'] = '#Cylinder'

            model.source =  meshMap[meshUri]
            wrapperNode.eulerRotation.x = 90
            wrapperNode.scale.x = root.cylinderRadius * 0.01 * 2
            wrapperNode.scale.y = root.cylinderLength * 0.01
            wrapperNode.scale.z = root.cylinderRadius * 0.01 * 2
            wrapperNode.position.z = root.cylinderLength * 0.1 / 2

        }
        else {

            cachedVisual.addMesh(encodeURIComponent(meshUri),
                                 `http://${client.hostname}:${client.port}/visual/get_mesh/${encodeURIComponent(meshUri)}`)



            // let callback = (msg) => {
            //     console.log(`recv mesh ${meshUri}`)
            //     geom.meshFile = msg
            //     model.geometry = geom
            // }

            // let meshFilename = `/visual/get_mesh/${encodeURIComponent(meshUri)}`

            // client.doRequestRaw('GET',
            //                     meshFilename,
            //                     '',
            //                     callback)
        }


    }

}

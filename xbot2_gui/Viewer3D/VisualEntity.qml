import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation

import "VisualEntity.js" as Logic

Entity {

    property string name: ''
    property string type: ''

    property alias meshSource: mesh.source
    property alias cylinderLength: cylinder.length
    property alias cylinderRadius: cylinder.radius
    property alias localRotation: tfLocal.rotation
    property alias localTranslation: tfLocal.translation

    property alias rotation: tfGlobal.rotation
    property alias translation: tfGlobal.translation
    property vector3d scale: "1, 1, 1"

    property alias alpha: material.alpha
    property alias color: material.diffuse
    property bool visible: true


    // private
    id: root
    property var activeMesh: Logic.getActiveMesh()
    components: [ activeMesh, material, tf ]

    Mesh {
        id: mesh
        source: ''
        onStatusChanged: (status) => {
            if(status === 3) {
                console.error(`[${name}] could not load mesh from ${source}`)
            }
        }
    }

    CylinderMesh {
        id: cylinder
    }

    Transform {
        id: tfMesh
        rotationX: root.type === 'CYLINDER' ? 90.0 : 0.0
        scale3D: root.scale.times(root.visible ? 1.0 : 0.0)
    }

    Transform {
        id: tfLocal
    }

    Transform {
        id: tfGlobal
    }

    Transform {
        id: tf
        matrix: tfGlobal.matrix.times(tfLocal.matrix).times(tfMesh.matrix)
    }

    PhongMaterial {
        id: material
        specular: Qt.lighter(diffuse)
        property real alpha: 0
    }

}

import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation

Entity {

    property url source: ''
    property alias rotation: tf.rotation
    property alias translation: tf.translation
    property alias scale: tf.scale3D
    property alias alpha: material.alpha
    property alias color: material.diffuse
    property bool visible: true

    id: root

    components: visible ? [ mesh, material, tf ] : []

    Mesh {
        id: mesh
        source: root.source
        onStatusChanged: (status) => {
            if(status === 3) {
                console.error(`could not load mesh from ${source}`)
            }
        }
    }

    Transform {
        id: tf
        scale3D: Qt.vector3d(0.001, 0.001, 0.001)*visible
    }

    PhongMaterial {
        id: material
        specular: Qt.lighter(diffuse)
        property real alpha: 0
    }

}

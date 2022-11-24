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

    id: root

    components: [ mesh, material, tf ]

    Mesh {
        id: mesh
        source: root.source
        onStatusChanged: {
            console.log('[Mesh] status: ', status)
        }
    }

//    SphereMesh {
//        id: mesh
//        radius: 0.6
//    }

    Transform {
        id: tf
        scale: 0.001
    }

    PhongMaterial {
        id: material
    }

}

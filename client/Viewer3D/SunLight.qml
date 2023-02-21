import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D

Entity {

    property alias translation: tf.translation
    property alias direction: light.worldDirection
    property alias intensity: light.intensity


    components: [
        DirectionalLight{
            id: light
            color: Qt.rgba(1, 1, 1, 1.0)
            intensity: 0.5
        },
        Transform{
            id: tf
            translation: Qt.vector3d(5,5,5)
        }
    ]
}

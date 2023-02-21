import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D

import ".."
import "RobotModel.js" as Logic

Node {

    property ClientEndpoint client

    property real alpha: 1.0

    property color color: 'red'

    function updateTf() {
        Logic.updateTf()
    }

    function updateModel() {
        Logic.updateModel()
    }


    // private
    id: root

    NodeInstantiator {

        id: visualRepeater

        model: 0

        delegate: VisualEntity {
            source: `http://${client.hostname}:${client.port}/visual/get_mesh/${encodeURIComponent(modelData.uri)}`
            scale: Qt.vector3d(modelData.scale[0],
                               modelData.scale[1],
                               modelData.scale[2])
            color: root.color
            alpha: root.alpha
        }
    }
}

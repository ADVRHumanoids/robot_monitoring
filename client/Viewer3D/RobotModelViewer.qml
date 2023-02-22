import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D

import xbot2_gui.RobotModel
import ".."
import "RobotModelViewer.js" as Logic

Node {

    property ClientEndpoint client

    property real alpha: 1.0

    property color color: 'red'

    property alias jointNames: model.jointNames

    property alias ndof: model.ndof

    function updateQ(q) {
        Logic.updateQ(q)
    }

    function updateTf() {
        Logic.updateTf()
    }

    function createViewer() {
        Logic.createViewer()
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

    RobotModel {
        id: model
    }
}

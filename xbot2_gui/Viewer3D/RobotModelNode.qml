import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D

import xbot2_gui.RobotModel
import Main
import "RobotModelNode.js" as Logic

Node {

    property ClientEndpoint client

    property real alpha: 1.0

    property color color: 'red'

    property alias jointNames: model.jointNames

    property alias ndof: model.ndof

    property bool visible: true

    property var q: Array(ndof).fill(0.0)

    signal modelChanged()

    onQChanged: updateQ(q)

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
            name: modelData.linkName
            type: modelData.type
            meshSource: modelData.type === 'MESH' ?
                            `http://${client.hostname}:${client.port}/visual/get_mesh/${encodeURIComponent(modelData.filename)}` :
                            ''
            cylinderLength: modelData.length || 0
            cylinderRadius: modelData.radius || 0
            scale: modelData.scale
            localTranslation: modelData.origin_xyz
            localRotation: modelData.origin_rot
            color: root.color
            alpha: root.alpha
            visible: root.visible
        }
    }

    RobotModel {
        id: model
        onModelChanged: root.modelChanged()
    }
}

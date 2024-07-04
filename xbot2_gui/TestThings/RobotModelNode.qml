import QtQuick
import QtQuick3D
import QtQuick3D.Helpers

import RobotModel
import Common
import Main
import ViewerQuick3D
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
        Logic.updateTf()
    }


    // private
    id: root

    Repeater3D {

        id: visualRepeater

        model: 0

        delegate: CustomMesh {
//            name: modelData.linkName
//            type: modelData.type
            meshUri: modelData.filename

            // cylinderLength: modelData.length || 0
            // cylinderRadius: modelData.radius || 0
            scale: modelData.scale
            localPosition: modelData.origin_xyz
            localRotation: modelData.origin_rot
            color: root.color
            alpha: root.alpha
            visible: root.visible
            client: root.client
        }
    }

    RobotModel {
        id: model
        onModelChanged: root.modelChanged()
    }

    Component.onCompleted: {
        createViewer()
    }
}

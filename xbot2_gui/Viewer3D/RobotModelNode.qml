import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D

import RobotModel
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

    function clearCache() {
        cachedVisual.clearCache()
    }


    // private
    id: root

    // dummy: just to be able to call clearCache()
    CachedVisual {
        id: cachedVisual
    }

    NodeInstantiator {

        id: visualRepeater

        model: 0

        delegate: VisualEntity {

            property string meshName: encodeURIComponent(modelData.filename)
            property string meshUrl: `http://${client.hostname}:${client.port}/visual/get_mesh/${meshName}`
            property alias cachedVisual: cachedVisual

            name: modelData.linkName
            type: modelData.type
            meshSource: type === 'MESH' ? cachedVisual.file : ''
            cylinderLength: modelData.length || 0
            cylinderRadius: modelData.radius || 0
            scale: modelData.scale
            localTranslation: modelData.origin_xyz
            localRotation: modelData.origin_rot
            color: root.color
            alpha: root.alpha
            visible: root.visible

            CachedVisual {
                id: cachedVisual
            }
        }

        onObjectAdded: function(idx, obj) {

            if(obj.type !== 'MESH') {
                return
            }

            obj.cachedVisual.addMesh(obj.meshName, obj.meshUrl)
        }
    }

    RobotModel {
        id: model
        onModelChanged: root.modelChanged()
    }
}

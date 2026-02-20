import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers

import Main
import Common
import ViewerQuick3D

import "../Main/sharedData.js" as SharedData

Item {

    id: root

    property ClientEndpoint client

    // ExpandableControl {
    //     z: 1
    //     // width: 300
    //     anchors {
    //         top: parent.top
    //         left: parent.left
    //         margins: 16
    //     }

    //     GridLayout {
    //         anchors.fill: parent
    //         columns: 2
    //         Label { text: 'Hey' }
    //         Button { text: 'Press Me' }
    //         Label { text: 'Hey' }
    //         Button { text: 'Press Me' }
    //         Label { text: 'Hey' }
    //         Button { text: 'Press Me' }
    //         Label { text: 'Hey' }
    //         Button { text: 'Press Me' }
    //         Label { text: 'Hey' }
    //         Button { text: 'Press Me' }
    //         Label { text: 'Hey' }
    //         Button { text: 'Press Me' }
    //     }
    // }

    Control {
        z: 1
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 16
        contentItem: ColumnLayout {

            Switch {
                text: 'Enable'
                onCheckedChanged: {
                    let msg_type = checked ? 'pc_registration' : 'pc_unregistration'
                    client.sendTextMessageUdp(JSON.stringify(
                                               {
                                                   'type': msg_type
                                               })
                                           )
                }
            }

            SpinBox {
                id: camHeightSpin
                from: 100
                to: 5000
                value: 400
            }

        }
    }

    // the main 3d node
    Node {

        id: standAloneScene

        Node {

            id: originNode

            PerspectiveCamera {
                id: cameraPerspectiveTwo
                clipNear: 1
                fieldOfView: 30
            }

            x: 0
            y: camHeightSpin.value
            z: 0
            eulerRotation.x: -90
            eulerRotation.y: -90

        }

        Node {

            // quick3d world (y-axis up)
            id: quickWorld

            DirectionalLight {
                ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
                brightness: 1.0
                eulerRotation.x: -25
            }

            Node {

                id: worldNode

                // robotic world (z-axis up)

                eulerRotation.x: -90

                Axes3D {
                    // mark origin with axes
                }

                Model {
                    // goal cuboid
                    id: goalMarker
                    source: "#Cube"
                    position: Qt.vector3d(0, 0, 0)
                    scale: Qt.vector3d(1, .5, .5)
                    materials: [
                        DefaultMaterial {
                            diffuseColor: Qt.hsva(0.8, 0.8, 0.8, 1)
                        }
                    ]
                    pickable: true

                    Model {
                        id: goalMarkerRing
                        geometry: TorusGeometry {

                        }
                        scale: Qt.vector3d(1, 2, 2)
                        eulerRotation.x: 90
                        materials: [
                            DefaultMaterial {
                                diffuseColor: Qt.hsva(0.2, 0.8, 0.8, 1)
                            }
                        ]
                        pickable: true
                    }
                }

                // RobotModelNode {
                //     client: root.client
                //     opacity: 0.5
                //     color: 'white'
                // }

                Repeater3D {

                    id: pcRepeater

                    model: 0

                    delegate: PointCloud {

                        property int iblkLast: -1

                        // required property var model

                        // position: Qt.vector3d(model.pos[0]*100,
                        //                       model.pos[1]*100,
                        //                       model.pos[2]*100)

                        // rotation: Qt.quaternion(model.rot[3],
                        //                         model.rot[0],
                        //                         model.rot[1],
                        //                         model.rot[2])
                    }
                }
            }
        }
    }


    View3D {

        anchors.fill: parent
        id: view3d
        importScene: standAloneScene
        camera: cameraPerspectiveTwo

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: Qt.rgba(0, 0, 0, 0)
            // InfiniteGrid {
            //     gridInterval: 30
            // }
        }

        MouseArea {
            anchors.fill: parent

            // interaction type
            property int interactionType: -1

            // for panning the scene
            property real lastX: -1
            property real lastY: -1

            // for dragging the goal marker
            property vector3d grabOffset
            property real depth

            // for rotating the goal marker
            property real angleOffset

            onPressed: function (mouse) {

                // pressed on goal marker?
                let result = view3d.pick(mouse.x, mouse.y)

                // yes, initiate goal marker dragging
                if (result.objectHit === goalMarker) {

                    depth = -cameraPerspectiveTwo.mapPositionFromScene(result.scenePosition).z
                    // grabOffset = goalMarker.scenePosition.minus(result.scenePosition)
                    interactionType = 1
                    return
                }

                // yes, initiate goal marker rotation
                if (result.objectHit === goalMarkerRing) {

                    depth = -cameraPerspectiveTwo.mapPositionFromScene(result.scenePosition).z
                    grabOffset = result.scenePosition.minus(goalMarker.scenePosition)
                    angleOffset = Math.atan2(grabOffset.z, grabOffset.x)
                    interactionType = 2
                    console.log(`start rotating from angle ${angleOffset}`)
                    return
                }

                // nope, pan the scene
                interactionType = 0
                lastX = mouse.x
                lastY = mouse.y
                console.log('started panning')
            }

            onPositionChanged: function (mouse) {

                // panning
                if(interactionType === 0) {
                    let dx = mouse.x - lastX
                    let dy = mouse.y - lastY

                    originNode.z -= dx * originNode.y / 800.0
                    originNode.x += dy * originNode.y / 800.0

                    lastX = mouse.x
                    lastY = mouse.y

                    return
                }

                // drag goal
                if(interactionType === 1) {

                    // mouse position in 3d scene at the depth of the goal marker
                    let worldMousePos = view3d.mapTo3DScene(
                            Qt.vector3d(mouse.x, mouse.y, depth)
                            )

                    console.log('mouse position', worldMousePos)

                    // desired global position of the goal marker
                    let desiredPosition = Qt.vector3d(
                                worldMousePos.x, // + grabOffset.x,
                                goalMarker.scenePosition.y,
                                worldMousePos.z, // + grabOffset.z
                                )

                    console.log('desired position (global)', desiredPosition)

                    let worldPosition = worldNode.mapPositionFromNode(standAloneScene, desiredPosition)

                    console.log('desired position (world)', worldPosition)

                    goalMarker.position = worldPosition

                    return
                }

                // rotate goal
                if(interactionType === 2) {
                    // mouse position in 3d scene at the depth of the goal marker
                    let worldMousePos = view3d.mapTo3DScene(
                            Qt.vector3d(mouse.x, mouse.y, depth)
                            )

                    // vector from goal marker to mouse position
                    grabOffset = worldMousePos.minus(goalMarker.scenePosition)

                    let angleNow = Math.atan2(grabOffset.z, grabOffset.x)

                    let angleDelta = angleOffset - angleNow

                    goalMarker.eulerRotation.z += angleDelta * 180 / Math.PI

                    angleOffset = angleNow
                }
            }

            onWheel: function (wheel) {
                camHeightSpin.value += wheel.angleDelta.y
            }


        }

        // OrbitCameraController {
        //     camera: cameraPerspectiveTwo
        //     origin: originNode
        //     anchors.fill: parent
        // }

    }

    property var pcNameToId: Object()
    property var sonarNameToId: Object()

    Connections {

        target: client

        function onPointCloudReceived(obj) {

            let pc = pcRepeater.objectAt(0)

            let instanceTable = pc.instancing

            // ideally a now point cloud starts with a iblk = 0 packet
            // in the case of packet loss, we may check if iblk is smaller than the last one
            // to detect point cloud start
            if(obj.iblk == 0 || obj.iblk < pc.iblkLast) {
                instanceTable.clear()
            }

            pc.iblkLast = obj.iblk

            // add all the points
            for(let i = 0; i + 2 < obj.xyz.length; i += 3) {
                instanceTable.addPoint(obj.xyz[i],
                                       obj.xyz[i+1],
                                       obj.xyz[i+2]
                                       )
            }

            // last iblk means we have received the whole point cloud
            // we can now update the screen
            if(obj.iblk == (obj.nblk - 1)) {
                instanceTable.commit()
            }
        }
    }
}

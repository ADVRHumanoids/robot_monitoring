import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebView

import Common
import Main
import ExpandableBottomBar
import Font
import Menu
import Joy

import QtQuick3D
import QtQuick3D.Helpers
import ViewerQuick3D

Rectangle {

    color: Qt.rgba(0.8, 0.8, 0.8, 1)

    property ClientEndpoint client

    property var pcNameToId: Object()
    property var sonarNameToId: Object()

    Connections {
        target: client
        function onObjectReceived(obj) {

            if(obj.type === 'pointcloud') {

                let instanceTable = pcRepeater.objectAt(pcNameToId[obj.name]).instancing

                instanceTable.clear()

                for(let i = 0; i < obj.points.length; i++) {
                    instanceTable.addPoint(obj.points[i][0],
                                           obj.points[i][1],
                                           obj.points[i][2]
                                           )
                }

                instanceTable.commit()

            }

            else if(obj.type === 'sonar') {

                console.log(JSON.stringify(obj.range))

                for(const [key, value] of Object.entries(obj.range)) {

                    sonarRepeater.objectAt(sonarNameToId[key]).radius = value

                }
            }

        }
    }

    Component.onCompleted: {

        client.doRequest('GET', '/visual/get_pointcloud', '',
                         function(res) {
                             let model = []
                             pcNameToId = {}
                             let i = 0
                             for(const [key, value] of Object.entries(res)) {
                                 model.push(value)
                                 pcNameToId[key] = i
                                 i += 1
                                 console.log(key)
                             }
                             pcRepeater.model = model
                             console.log(JSON.stringify(model))
                         })


        client.doRequest('GET', '/visual/get_sonar', '',
                         function(res) {
                             let model = []
                             sonarNameToId = {}
                             let i = 0
                             for(const [key, value] of Object.entries(res)) {
                                 model.push(value)
                                 sonarNameToId[key] = i
                                 i += 1
                                 console.log(key)
                             }
                             sonarRepeater.model = model
                             console.log(JSON.stringify(model))
                         })
    }

    //
    id: root

    // The root scene
    Node {

        id: standAloneScene

        Node {

            id: originNode

            PerspectiveCamera {
                id: cameraPerspectiveTwo
                z: 200
                clipNear: 1
            }

            x: 0
            y: 400
            z: 0
            eulerRotation.x: -90
            eulerRotation.y: -90

        }

        Node {

            DirectionalLight {
                ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
                brightness: 1.0
                eulerRotation.x: -25
            }

            Node {


                eulerRotation.x: -90

                Repeater3D {

                    id: pcRepeater

                    delegate: PointCloud {

                        required property var model

                        position: Qt.vector3d(model.pos[0]*100,
                                              model.pos[1]*100,
                                              model.pos[2]*100)

                        rotation: Qt.quaternion(model.rot[3],
                                                model.rot[0],
                                                model.rot[1],
                                                model.rot[2])
                    }
                }

                Repeater3D {

                    id: sonarRepeater

                    delegate: Model {

                        id: sonarDelegate

                        required property var model

                        property real radius: 1.0

                        source: '#Sphere'

                        scale: Qt.vector3d(radius, radius, radius)

                        materials: PrincipledMaterial {
                            baseColor: 'red'
                            opacity: Math.min(Math.max(0, (0.30 - sonarDelegate.radius)/(0.75 - 0.30) + 1.0), 1)
                        }

                        position: Qt.vector3d(model.pos[0]*100,
                                              model.pos[1]*100,
                                              model.pos[2]*100)

                        rotation: Qt.quaternion(model.rot[3],
                                                model.rot[0],
                                                model.rot[1],
                                                model.rot[2])
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
            clearColor: Qt.rgba(0.8, 0.8, 0.8, 1)
            InfiniteGrid {
                gridInterval: 30
            }
        }

        OrbitCameraController {
            camera: cameraPerspectiveTwo
            origin: originNode
            anchors.fill: parent
        }

        MouseArea {
            anchors.fill: parent
            onClicked: (mouse) => {
                           var result = view3d.pick(mouse.x, mouse.y);
                           var pickedObject = result.objectHit;
                           pickedObject.isPicked = !pickedObject.isPicked;
                       }
        }
    }
}


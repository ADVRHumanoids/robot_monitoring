import QtCore
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import QtQuick.Controls.Material

import Main
import ViewerQuick3D

import "./SandingComponents"
import "Sanding3D.js" as Logic

Item {
    id: root
    property ClientEndpoint client
    property vector3d mapPosition
    property quaternion mapOrientation
    property string currentStatus: "Ready to Scan"

    function reset() {
        for (var i = 0; i < wallRepeater.count; i++) {
            wallRepeater.children[i].isPicked = false
        }
        wallRepeater.ready = false
        startSendingButton.readyToSand = false
        windowSettings.source = ""
    }

    function removeWall() {
        console.log("Removing Walls...")
        currentStatus = "Ready to Scan"
        wallList.clear()
    }


    // property var wallList: []
    Node {

        id: standAloneScene

        Node {

            id: originNode

            PerspectiveCamera {
                id: cameraPerspectiveTwo
                z: 200
                clipNear: 1
            }

            x: 250
            y: 350
            z: 250
            eulerRotation.y: 40
            eulerRotation.x: -40
        }

        Node {

            id: modelScene

            DirectionalLight {
                ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
                brightness: 1.0
                eulerRotation.x: -25
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
                 clearColor: "#24273a"
                 InfiniteGrid {
                     gridInterval: 100
                 }
             }

        OrbitCameraController {
            camera: cameraPerspectiveTwo
            origin: originNode
            anchors.fill: parent
        }

        Spinner {
            id: scanningLoader
            anchors.centerIn: parent
            visible: false

            Text {
                id: scanningProgress
                anchors.centerIn: parent
                font.bold: false
                font.pixelSize: 27
                font.letterSpacing: 1
                opacity: enabled ? 1.0 : 0.3
                color: scanningLoader.loaderColor
                horizontalAlignment: parent.AlignHCenter
                verticalAlignment: parent.AlignVCenter
                elide: Text.ElideRight
            }
        }

        RobotModelNode {
            id: concertModel
            client: root.client
            position: mapPosition
            rotation: mapOrientation
            eulerRotation.x: -90
        }

        ListModel {
            id: wallList
        }

        Repeater3D {
            id: wallRepeater
            model: wallList
            property bool ready: false // ready when one and one only is selected
            onModelChanged: {
                wallRepeater.update()
            }

            delegate: Model {
                property int idx
                property int index
                property bool isPicked: false
                property bool selected: false
                property string type

                id: panel
                visible: true
                source: "#Cube"
                // scale.x: 0.2
                // scale.y: model.l
                scale: Qt.vector3d(model.l, 0.2, 1) // [lenght, depth, height]
                pickable: true

                materials: DefaultMaterial {
                    diffuseColor: panel.isPicked ?  "#e78284" : "#ca9ee6"
                    specularAmount: 1
                    specularRoughness: 1
                }

                // scale.z: 0.2
                state: selected
                // eulerRotation.x: -90
                idx : model.id
                index: model.index
                position: model.position
                rotation: model.orientation
                type: model.type
                // eulerRotation: Qt.vector3d(-90, 0, 0)
            }
        }

        MouseArea{
            id: mouseArea
            property QtObject instance
            // property QtObject instance
            anchors.fill: parent
            onClicked: (mouse) => {

                var result = view3d.pick(mouse.x, mouse.y);
                var panel = result.objectHit;
                var component = Qt.createComponent("SandingSendingButton.qml")
                // var interactiveComponent= Qt.createComponent("InteractivePanel.qml")
                // interactive = interactiveComponent.createObject(panel, {"panel": panel})
                // if (!panel.isPicked && !panel.parent.ready && !sendButton.show ) {
                if (!panel.isPicked && !panel.parent.ready) {
                    // focusPanel()
                    // menu = component.createObject(standAloneScene)
                    // if (!panel.parent.ready) {
                    panel.parent.ready = true
                    panel.isPicked = !panel.isPicked
                    // }
                    // wallModel.setState(panel.idx)

                    addSetting(panel)
                }
                else if (panel.isPicked && panel.parent.ready) {
                    reset(jointNames)
                    // panel.parent.ready = false
                    // sendButton.show = false
                    // wallModel.setState(panel.idx)
                    // panel.isPicked = !panel.isPicked
                    // deleteSetting()
                }
            }

            onDoubleClicked: {
                // reset()
                // deleteSetting()
            }

            function addSetting(panel) {
                windowSettings.source  = "SandingComponents/SandingSettings.qml"
                windowSettings.item.panelWidth = panel.scale.x
                windowSettings.item.patch["ID"] = panel.idx
                windowSettings.item.patch["index"] = panel.index
                windowSettings.item.patch["y"] = panel.index + 1
                windowSettings.item.patch["type"] = panel.type
                console.log("Selecting panel ID: " + panel.idx + " with index: " + panel.index)
            }
        }

        Text {
            id: statusText
            text: "Status: " + currentStatus
            font.pixelSize: 27
            font.letterSpacing: 1
            // anchors to the top left corner
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.topMargin: 5
            anchors.leftMargin: 5
            color: "#cad3f5"
        }

        ScanButton {
            id: scanningButton
            anchors.centerIn: parent
        }

        StartSendingButton {
            id: startSendingButton
            anchors.centerIn: parent
        }

        StartToolButton {
            id: startToolButton
            visible: currentStatus === 'Waiting'
            anchors.centerIn: parent
        }

        ResetButton {
            // anchors.centerIn: parent
            // Layout.alignment:Qt.AlignCenter
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
        }

        // SandingSendingButton {
        //     id: sendButton
        //     anchors.bottom: parent.bottom
        //     anchors.horizontalCenter: parent.horizontalCenter
        // }

        // SandingSettings {
        //     id: settings
        // }

        // FIX use Instantiator
        Loader {
            id: windowSettings
            // source: "SandingComponents/SandingSettings.qml"
            // onLoaded: {
            //     if (windowSettings.item) {

            //     }
            // }
        }
        Connections {
            target: windowSettings.item
            function onClose() {
                reset()
                // console.log("Closing")
                // windowSettings.source = ""
            }
        }
    }

    Connections {
        target: client

        function onJointStateReceived(js) {
            Logic.jsCallback(js)
        }

        onObjectReceived: function(msg) {

            if(msg.type === 'scanning_progress') {
                // console.log("Percent: ", msg.progress)
                // statusText.text = msg.status
                // progressBar.value = msg.progress
                // progressBar.indeterminate = msg.progress < 0
                scanningProgress.text = msg.progress + "%"
                if(msg.progress >= 100) {
                    // statusText.text = 'Completed'
                    scanningLoader.visible = false
                    // deactivate the panel
                }
            }
            if (msg.type === 'wall_list') {
                // wallList = []
                console.log("Getting Wall List!")
                for (let w of msg.walls) {
                    let obj = Logic.toQMLObject(w)
                    wallList.append(obj)
                    console.log("Found wall :  ", obj.id)
                    console.log("Position: ", obj.position)
                    console.log("Orientation: ", obj.orientation)
                    console.log("Length: ", obj.l)
                    console.log("Index: ", obj.index)
                    console.log("Type: ", obj.type)
                }
            }
            if (msg.type === 'map') {
                Logic.updateConcertPose(msg.transform)
            }

            if (msg.type === 'concert_sanding_progress') {
                currentStatus = msg.status
                console.log("Current Status " + msg.status)
                if (msg.progress >= 100) {
                    currentStatus = "Completed"
                }
            }
        }
    }

    Component.onCompleted: {
        Logic.construct()
    }



}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Qt3D.Core
import Qt3D.Render
import Qt3D.Input
import Qt3D.Logic
import Qt3D.Extras
import Qt3D.Animation
import QtQuick.Scene2D
import QtQuick.Scene3D


import "../Viewer3D"
import ".."
import "../sharedData.js" as SharedData
import "RobotModelViewer.js" as Logic

Viewer3D {

    property ClientEndpoint client
    property alias robotState: robotState
    property alias robotCmd: robotCmd

    function updateRobotState(js, robot, fieldName) {
        Logic.updateViewerState(js, robot, fieldName)
    }

    function resetCmd() {
        Logic.updateViewerState(SharedData.latestJointState,
                                robotCmd,
                                'posRef')
    }


    // private
    id: root

    nodes: [
        RobotModelNode {
            id: robotState
            client: root.client
            visible: stateCheck.checked
            color: 'white'

            onModelChanged: {
                Logic.updateViewerState(SharedData.latestJointState,
                                        robotState,
                                        'linkPos')
            }
        },

        RobotModelNode {
            id: robotCmd
            client: root.client
            visible: cmdCheck.checked
            color: 'red'

            onModelChanged: {
                Logic.updateViewerState(SharedData.latestJointState,
                                        robotCmd,
                                        'posRef')
            }
        }
    ]

    GridLayout {

        id: ctrlGrid

        anchors {
            top: parent.top
            bottom: parent.bottom
            left: parent.left
            margins: 0
        }

        columns: 2

        columnSpacing: 8
        rowSpacing: 4

        CheckBox {
            id: stateCheck
            text: 'Show state robot'
            Layout.columnSpan: 2
            checked: true
            checkable: true
        }

        CheckBox {
            id: cmdCheck
            text: 'Show command robot'
            Layout.columnSpan: 2
            checked: true
            checkable: true
        }

        Button {
            Layout.columnSpan: 2
            text: 'Reload'
            onClicked: {
                robotState.createViewer()
                robotCmd.createViewer()
            }
        }

        Item {
            Layout.fillHeight: true
        }
    }

    Component.onCompleted: {
        robotCmd.createViewer()
        robotState.createViewer()
    }
}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Main
import "/qt/qml/Main/sharedData.js" as SharedData

Control {

    property ClientEndpoint client
    property bool expanded: true

    id: root

    property real maxTempMot: 0
    property real maxTempDri: 0
    property string jnameMaxTempMot: ''
    property string jnameMaxTempDri: ''
    property real vBatt: 0
    property real danger: Math.min(1, Math.max( 0, (51 - vBatt)/4, (maxTempMot - 50)/30, (maxTempDri - 50)/10 ))

    property color okColor: 'green'
    property color badColor: 'red'

    padding: 4



    background: Rectangle {
        id: bg
        color: Qt.rgba(
                   (1-danger)*okColor.r + danger*badColor.r,
                   (1-danger)*okColor.g + danger*badColor.g,
                   (1-danger)*okColor.b + danger*badColor.b,
                   1.0
                   )
        radius: root.expanded ? 4 : width/2
    }

    contentItem: Item {

        implicitWidth: grid.visible ? grid.implicitWidth : 30
        implicitHeight: grid.visible ? grid.implicitHeight : 30

        Behavior on implicitWidth {
            NumberAnimation {
                duration: 333
                easing.type: Easing.OutQuad
            }
        }
        Behavior on implicitHeight {
            NumberAnimation {
                duration: 333
                easing.type: Easing.OutQuad
            }
        }

        MouseArea {
            anchors.fill: parent
            onClicked: {
                root.expanded = !root.expanded
            }
            z: 1
        }

        Rectangle {
            width: parent.width / 2
            height: width
            radius: width/2
            color: Qt.rgba(0, 0, 0, 0.1)
            anchors.centerIn: parent
            visible: !grid.visible
        }

        GridLayout {
            id: grid
            anchors.fill: parent
            visible: root.expanded
            columns: 2

            Column {

                Layout.fillWidth: true
                Layout.columnSpan: 2

                Label {
                    text: 'Vbatt'
                    font.pointSize: 6
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: `${vBatt} V`
                    anchors.horizontalCenter: parent.horizontalCenter
                }

            }

            Column {

                Layout.preferredWidth: 40
                Layout.fillWidth: true

                Label {
                    text: 'Driver'
                    font.pointSize: 6
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: `${root.maxTempDri}°`
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: root.jnameMaxTempDri
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pointSize: 6
                }

            }

            Column {

                Layout.preferredWidth: 40
                Layout.fillWidth: true

                Label {
                    text: 'Motor'
                    font.pointSize: 6
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: `${root.maxTempMot}°`
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: root.jnameMaxTempMot
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pointSize: 6
                }

            }
        }

    }

    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            let js = SharedData.latestJointState
            root.vBatt = js.vbatt
            root.maxTempMot = Math.max( ...js.motorTemp )
            root.maxTempDri = Math.max( ...js.driverTemp )
            jnameMaxTempMot = SharedData.jointNames[ js.motorTemp.indexOf(root.maxTempMot) ]
            jnameMaxTempDri = SharedData.jointNames[ js.driverTemp.indexOf(root.maxTempDri) ]
        }
    }

    Connections {
        target: client
    }
}

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
    property real danger: Math.min(1, Math.max( 0,
                                               (vBatt > 0)*(51 - vBatt)/4,
                                               (maxTempMot > 0)*(maxTempMot - 50)/30,
                                               (maxTempDri > 0)*(maxTempDri - 50)/10 ))

    property color okColor: Qt.hsva(0.333, 0.8, 1.0, 1.0)
    property color badColor: Qt.hsva(0.0, 0.8, 1.0, 1.0)

    padding: 4



    background: Rectangle {
        id: bg
        property color baseColor: Qt.rgba(
                   (1-danger)*okColor.r + danger*badColor.r,
                   (1-danger)*okColor.g + danger*badColor.g,
                   (1-danger)*okColor.b + danger*badColor.b,
                   1.0
                   )
        color: mouse.containsMouse ? Qt.lighter(baseColor, 1.25) : baseColor
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
            id: mouse
            anchors.fill: parent
            onClicked: {
                root.expanded = !root.expanded
            }
            z: 1
            hoverEnabled: true
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

                Layout.alignment: Qt.AlignTop
                Layout.fillWidth: true
                Layout.columnSpan: 2

                visible: vBatt > 0

                Label {
                    text: 'Vbatt'
                    font.pointSize: 6
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: `${vBatt.toFixed(1)} V`
                    anchors.horizontalCenter: parent.horizontalCenter
                }

            }

            Column {

                Layout.alignment: Qt.AlignTop
                Layout.preferredWidth: 40
                Layout.fillWidth: true

                Label {
                    text: 'Driver'
                    font.pointSize: 6
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: `${root.maxTempDri.toFixed(1)}°`
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: root.jnameMaxTempDri
                    anchors.horizontalCenter: parent.horizontalCenter
                    wrapMode: Text.WrapAnywhere
                    font.pointSize: 6
                    width: parent.width
                    horizontalAlignment: Text.AlignHCenter
                }

            }

            Column {

                Layout.alignment: Qt.AlignTop
                Layout.preferredWidth: 40
                Layout.fillWidth: true

                Label {
                    text: 'Motor'
                    font.pointSize: 6
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: `${root.maxTempMot.toFixed(1)}°`
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Label {
                    text: root.jnameMaxTempMot
                    anchors.horizontalCenter: parent.horizontalCenter
                    wrapMode: Text.WrapAnywhere
                    font.pointSize: 6
                    width: parent.width
                    horizontalAlignment: Text.AlignHCenter
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

            if(js === undefined) {
                return
            }

            root.vBatt = js.vbatt ?? 0
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

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Font

Item {

    property string jointName: "joint_name"
    property alias bar: bar
    property real labelColorAlpha: (isSelected || labelMouseArea.containsMouse) ? 0.6 : 0.3
    property alias labelMouseArea: labelMouseArea
    property bool isSelected: false
    property int motorStatus: -1
    property bool brakeStatus: false

    signal jointClicked(string jName)

    function setStatus(ok) {
        _statusOk = ok
        statusTimer.restart()
    }

    function motorStatusAsColor(st) {

        // bitmask:
        // LSB  : on
        // bit-1: ready to switch on
        // bit-2: fault

        let _on = (st & 0x1) !== 0
        let _rtso = (st & 0x2) !== 0
        let _fault = (st & 0x4) !== 0

        if(_on) return 'green'
        if(_rtso) return 'yellow'
        if(_fault) return 'red'
        return 'gray'
    }


    // private
    id: root
    implicitHeight: row.implicitHeight
    implicitWidth: row.implicitWidth

    property bool _statusOk: true

    RowLayout {

        id: row
        anchors.fill: parent

        Control {

            id: leftCtrl

            padding: 4

            background: Rectangle {
                border.color: "red"
                color: _statusOk ?
                           Qt.rgba(0.9, 0.9, 0.9, labelColorAlpha) :
                           Qt.rgba(1, 0, 0, labelColorAlpha)
                radius: 3
                border.width: _statusOk ? 0 : 1
            }

            contentItem: RowLayout {

                Rectangle {

                    visible: root.motorStatus >= 0
                    radius: 3
                    color: motorStatusAsColor(root.motorStatus)
                    Layout.preferredWidth: 6
                    Layout.preferredHeight: 6

                    Label {
                        text: MaterialSymbolNames.lock
                        z: 1
                        font.family: syms.font.family
                        font.pixelSize: 14
                        color: 'black'
                        anchors.centerIn: parent
                        visible: root.brakeStatus
                    }

                }

                Label {
                    id: jointNameLabel
                    text: jointName

                    font.pixelSize: Qt.application.font.pixelSize * 0.9
                    wrapMode: Text.Wrap
                    Layout.preferredWidth: 100

                    MouseArea {
                        id: labelMouseArea
                        anchors.fill: parent
                        hoverEnabled: true

                        onClicked: {
                            jointClicked(jointName)
                        }
                    }
                }

            }
        }


        TwoSideBar {
            id: bar
            Layout.fillWidth: true
            height: leftCtrl.height
        }

    }

    Timer {
        id: statusTimer
        interval: 500
        onTriggered: {
            _statusOk = true
        }
    }

}

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

    property bool _rtso: motorStatus & 0x1
    property bool _on: motorStatus & 0x4
    property bool _fault: motorStatus & 0x8

    signal jointClicked(string jName)

    function setStatus(ok) {
        _statusOk = ok
        statusTimer.restart()
    }

    function motorStatusAsColor(st) {

        // bitmask:
        // w	sod	qs	ve	f	oe	so	rtso

        if(_on) return Qt.hsva(0.3, 0.8, 1.0, 1.0)  // green if motor on
        if(!_rtso) return Qt.hsva(0.166, 0.8, 1.0, 1.0)  // yellow if not ready to switch on
        if(_fault) return Qt.hsva(0., 0.8, 1.0, 1.0)  // red if fault
        return 'gray'  // gray if off but otherwise ok and ready
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
                        color: Qt.hsva(0., 0.8, 7.0, 1.0)
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
            enabled: root.motorStatus < 0 || root._on
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

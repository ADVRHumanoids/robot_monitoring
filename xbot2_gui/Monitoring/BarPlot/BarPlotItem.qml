import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {

    property string jointName: "joint_name"
    property alias bar: bar
    property real labelColorAlpha: 0.3
    property alias labelMouseArea: labelMouseArea

    signal jointClicked(string jName)

    function setStatus(ok) {
        _statusOk = ok
        statusTimer.restart()
    }


    // private
    id: root
    implicitHeight: row.implicitHeight
    implicitWidth: row.implicitWidth

    property bool _statusOk: true

    RowLayout {

        id: row
        anchors.fill: parent

        Label {
            id: jointNameLabel
            text: jointName
            background: Rectangle {
                border.color: "red"
                color: _statusOk ? Qt.rgba(0.9, 0.9, 0.9, labelColorAlpha) : Qt.rgba(1, 0, 0, labelColorAlpha)
                radius: 3
                border.width: _statusOk ? 0 : 1
            }
            font.pixelSize: Qt.application.font.pixelSize * 0.9
            wrapMode: Text.Wrap
            Layout.preferredWidth: 100
            padding: 5

            MouseArea {
                id: labelMouseArea
                anchors.fill: parent
                hoverEnabled: true

                onHoveredChanged: {
                    if(labelMouseArea.containsMouse)
                    {
                        labelColorAlpha = 0.6
                    }
                    else
                    {
                        labelColorAlpha = 0.3
                    }
                }

                onClicked:  jointClicked(jointName)
            }
        }

        TwoSideBar {
            id: bar
            Layout.fillWidth: true
            height: jointNameLabel.height
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

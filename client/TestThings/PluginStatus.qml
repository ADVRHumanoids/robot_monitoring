import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

Rectangle {
    id: root
    property bool pluginRunning: true
    property string pluginName: 'Homing'
    implicitHeight: mainCol.implicitHeight + 32
    color: Qt.lighter(Material.background)
    radius: 4

    Column {

        id: mainCol
        width: parent.width - 32
        anchors.centerIn: parent
        spacing: 16

        RowLayout {

            id: upperRow
            width: parent.width
            spacing: 16


            Label {
                id: nameLabel
                text: root.pluginName
                font.bold: root.pluginRunning
                font.pixelSize: 16
                color: root.pluginRunning ? Material.color(Material.Green) :
                                            Material.primaryTextColor
            }


            Button {
                id: startStopBtn
                text: 'Start'
                Layout.fillWidth: true
            }

            Button {
                id: abortBtn
                text: 'Abort'
                Layout.fillWidth: true
            }

        }

        RowLayout {

            width: parent.width

            Label {
                text: 'CPU time'
                Layout.alignment: Qt.AlignVCenter
            }

            ProgressBar {
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignVCenter
            }

        }

    }

}

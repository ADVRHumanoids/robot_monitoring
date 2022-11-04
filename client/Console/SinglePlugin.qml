import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

Item {

    id: root
    implicitHeight: row.implicitHeight
    implicitWidth: row.implicitWidth

    property alias name: nameLabel.text
    property string state: "None"
    property real period: 0.0
    property alias cputime: cpuBar.value

    signal start
    signal stop
    signal abort

    onCputimeChanged: {
        cpuBar.indeterminate = false
    }

    RowLayout {

        id: row
        spacing: 5

        anchors.fill: parent

        Label {
            id: nameLabel
            Layout.preferredWidth: 80
            Layout.alignment: Qt.AlignVCenter
        }

        Button {
            id: startStopBtn
            text: (root.state === "Stopped" || root.state === "Initialized") ? "Start" : "Stop"
            Layout.alignment: Qt.AlignVCenter
            onReleased: {
                if(text === "Start") start()
                if(text === "Stop") stop()
            }
        }

        Button {
            id: abortBtn
            text: "Abort"
            Layout.alignment: Qt.AlignVCenter
            onReleased: {
                abort()
            }
        }

        ProgressBar {
            id: cpuBar
            from: 0
            to: root.period
            indeterminate: true
            Layout.alignment: Qt.AlignVCenter
            Layout.fillWidth: true

            Label {
                anchors.centerIn: parent
                text: (cputime*1000).toFixed(2) + ' ms'

                Rectangle {
                    color: "white"
                    opacity: 0.2
                    radius: 4
                    anchors.centerIn: parent
                    width: parent.width + 6
                    height: parent.height + 6
                }
            }


        }

        TextEdit {
            readOnly: true
            text: root.state
            Layout.alignment: Qt.AlignVCenter
            Layout.preferredWidth: 40
            color: Material.primaryTextColor
        }

    }

}

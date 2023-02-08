import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Item {

    Card1 {

        id: card
        anchors.centerIn: parent

        name: 'ecat_master'

        width: 400
        defaultHeight: 400

        frontItem: ScrollView {
            anchors.fill: parent
            contentHeight: rec.height
            contentWidth: rec.width
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOn
            ScrollBar.vertical.policy: ScrollBar.AlwaysOn
            Rectangle {
                id: rec
                width: 300
                height: 600
                color: Qt.rgba(1, 0, 0, 0.1)
                border.color: 'red'
            }
        }

        backItem: ScrollView {
            anchors.fill: parent
            contentHeight: col.implicitHeight
            Column {
                id: col
                anchors.horizontalCenter: parent.horizontalCenter
                Repeater {

                    model: 10

                    Button {
                        text: 'Button ' + index
                    }
                }
            }
        }
    }



}

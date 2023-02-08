import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Item {

    Card1 {

        id: card
        anchors.centerIn: parent

        name: 'ecat_master'

        hidden: true

        width: 400
//        defaultHeight: 400

        frontItem: ScrollView {
            enabled: !card.hidden
            width: parent.width
            height: Math.min(contentHeight, 400)
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
            width: parent.width
            height: Math.min(contentHeight, 400)
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

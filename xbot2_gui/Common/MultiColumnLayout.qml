import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Common

Item {

    property int columns

    //
    id: root

    property int _nitems: 0

    ScrollView {

        anchors.fill: parent

        RowLayout {

            spacing: 24

            Repeater {

                model: root.columns

                Column {

                }
            }
        }

    }

    onChildrenChanged: {
        children[i]
    }

}

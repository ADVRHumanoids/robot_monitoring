import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common
import Font

Control {

    property alias text: title.text
    property alias titleFont: title.font
    signal clicked()
    property alias iconText: iconLabel.text
    property int pixelSize: CommonProperties.font.h2
    default property alias data: moreItemsRow.data

    padding: 6
    background: Rectangle {
        border.color: Qt.rgba(1, 1, 1, 0.04)
        border.width: 2
        color: 'transparent'
        radius: 4
        visible: grid.columns === 1
    }

    //
    id: root

    contentItem: GridLayout {

        id: grid

        columns: width > 300 ? 2 : 1

        RowLayout {

            Layout.fillWidth: true

            Label {

                id: iconLabel

                font.family: CommonProperties.fontAwesome.solid.family
                font.pixelSize: root.pixelSize
                visible: text !== ''

                MouseArea {
                    anchors.fill: parent
                    onClicked: root.clicked()
                }

            }

            Label {
                id: title
                text: 'Title'
                font.pixelSize: root.pixelSize
                Layout.fillWidth: true
                wrapMode: Text.Wrap

                MouseArea {
                    anchors.fill: parent
                    onClicked: root.clicked()
                }
            }

        }

        RowLayout {
            id: moreItemsRow
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignRight
        }
    }

}

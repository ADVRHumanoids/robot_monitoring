import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import xbot2_gui.common

RowLayout {

    id: root
    property alias text: title.text

    Label {
        id: title
        text: 'Title'
        font.pixelSize: CommonProperties.font.h1
    }

    Item {
        Layout.fillWidth: true
    }

}

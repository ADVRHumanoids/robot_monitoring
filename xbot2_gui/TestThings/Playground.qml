import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebView

import Common
import Main
import ExpandableBottomBar
import Font
import Menu
import Joy

Item {

    property ClientEndpoint client

    id: root

    TabBar {
        id: bar
        anchors.top: parent.top
        width: parent.width

        TabButton {
            text: 'Motion'
        }

        TabButton {
            text: 'Calibration'
        }
    }

    StackLayout {
        anchors.top: bar.bottom
        anchors.bottom: parent.bottom
        width: parent.width

        MotionTab {
            client: root.client
        }
    }



}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main
import ExpandableBottomBar
import Font
import Menu

MultiPaneResponsiveLayout {

    property ClientEndpoint client




    Repeater {
        model: 3
        Rectangle {
            property string iconText: 'Miao ' + index
            required property int index
            border.color: 'blue'
            border.width: 4
            color: palette.active.accent
//            height: 100
//            width: 100
//            implicitWidth: 100
//            implicitHeight: 100



        }
    }


    ScrollView {

        id: scroll
        contentWidth: availableWidth

        Card1 {

            width: scroll.availableWidth

            frontItem: Rectangle {

                width: parent.width
                height: 500

            }

        }

    }





}


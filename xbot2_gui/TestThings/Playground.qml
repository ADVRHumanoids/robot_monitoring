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


    Timer {
        running: true
        interval: 16
        repeat: true
        property int t: 0
        property var series

        onTriggered: {

            if(CommonProperties.globalLivePlot === null)
            {
                return
            }

            if(t === 0) {
                series = CommonProperties.globalLivePlot.addSeries('test', Object())
            }

            t += interval

            CommonProperties.globalLivePlot.addPoint(series, t/1000, Math.sin(t/1000.))

        }
    }

}


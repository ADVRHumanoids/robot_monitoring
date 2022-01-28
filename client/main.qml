import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import QtQuick.Shapes 1.14

import "SingleJointState"
import "BarPlot"
import "sharedData.js" as SharedData

Window {

    id: mainWindow
    width: 360
    height: 568
    visible: true
    title: "Xbot2 Robot GUI"

    BarPlotStack {
        id: barPlot
        anchors.fill: parent
    }

    ClientEndpoint {
        id: client
        onError: function (msg) {
            // hello.setError(msg)
        }
        onConnected: function (msg) {
            // hello.setConnected(msg)
        }
        onJointStateReceived: function (msg) {
            barPlot.setJointStateMessage(msg)
        }
        onFinalized: {
            barPlot.construct()
        }
    }
}

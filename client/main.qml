import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import QtQuick.Shapes 1.14

import "SingleJointState"
import "BarPlot"

Window {

    id: mainWindow
    width: 360
    height: 568
    visible: true
    title: "Xbot2 Robot GUI"

    BarPlot {
        id: barPlot
    }

    ClientEndpoint {
        id: client
        onError: function (msg) {
            hello.setError(msg)
        }
        onConnected: function (msg) {
            hello.setConnected(msg)
        }
        onJointStateReceived: function (msg) {
            barPlot.setJointStateMessage(msg)
        }
        onFinalized: {
            // hello.setProgress('constructing UI, this could take a while..')

            barPlot.min = qmin
            barPlot.max = qmax
            barPlot.jointNames = jointNames

            // singleJointState.construct(jointNames)
        }
    }
}

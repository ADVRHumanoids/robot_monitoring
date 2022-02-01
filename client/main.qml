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
    width: 640
    height: 480
    visible: true
    title: "Xbot2 Robot GUI"

    RowLayout
    {

        anchors.fill: parent
        anchors.margins: 8

        BarPlotStack {
            id: barPlot
            Layout.fillHeight: true
            Layout.fillWidth: true

            onJointClicked: function(jn)
            {
                jointState.selectJoint(jn)
            }
        }

        SingleJointStateStack {
            id: jointState
            Layout.fillHeight: true

            onProgressChanged: function(msg) {
                hello.setProgress(msg)
            }
        }

    }

    HelloScreen {
        id: hello
        anchors.fill: parent

        Behavior on opacity {
            NumberAnimation {
                duration: 1000
                onRunningChanged: {
                    hello.visible = hello.opacity > 0
                }
            }
        }
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
            jointState.setJointStateMessage(msg)
        }
        onFinalized: {
            barPlot.construct()
            jointState.construct()
            hello.opacity = 0
        }
    }
}

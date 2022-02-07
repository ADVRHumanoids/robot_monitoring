import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import QtQuick.Shapes 1.14

import "SingleJointState"
import "BarPlot"
import "sharedData.js" as SharedData
import "main.js" as Main

ApplicationWindow {

    id: mainWindow
    width: 400
    height: 480
    visible: true
    title: "Xbot2 Robot GUI"

    property string layoutMode: ""

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
    }

    SwipeView {
        id: swipeView
        anchors.fill: parent
        anchors.margins: 8
        clip: true
    }

    RowLayout
    {
        id: rowLayout
        anchors.fill: parent
        anchors.margins: 8
    }

    footer: ToolBar {
        PageIndicator {
        anchors.centerIn: parent
        count: swipeView.count
        currentIndex: swipeView.currentIndex
    }
    }

    onWidthChanged: {
        Main.handleResponsiveLayout()
    }

    Component.onCompleted: {
        Main.handleResponsiveLayout()
    }

    onLayoutModeChanged: Main.setLayoutMode(layoutMode)

    HelloScreen {
        id: hello
        anchors.fill: parent

        Behavior on opacity {
            NumberAnimation {
                duration: 1000
                onRunningChanged: {
                    print('run anim')
                    hello.visible = hello.opacity > 0
                }
            }
        }

        onUpdateServerUrl: function(hostname, port) {
            client.hostname = hostname
            client.port = port
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

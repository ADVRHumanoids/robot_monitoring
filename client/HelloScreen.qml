import QtQuick 2.4
import QtQuick.Controls.Material

HelloScreenForm {

    id: hello

    property ClientEndpoint client: undefined

    signal updateServerUrl(var host, var port)

    function setError(msg) {
        msgText.text = "Error: " + msg
        msgText.color = Material.color(Material.Red)
    }

    function setConnected(msg) {
        msgText.text = "Status OK: " + msg
        msgText.color = Material.color(Material.Green)
    }

    function setProgress(msg) {
        statusText.text = msg
    }

    Behavior on opacity {
        NumberAnimation {
            duration: 1000
            onRunningChanged: {
                hello.visible = hello.opacity > 0.1
            }
        }
    }

    applyBtn.onReleased: {
        client.hostname = serverHost
        client.port = serverPort
        client.active = true
    }

    onWidthChanged: {
        if(width < 360)
        {
            mainLayout.anchors.margins = 20
        }
        else
        {
            mainLayout.anchors.margins = 50
        }
    }

    Component.onCompleted: {
        print('hello built!!')
    }
}

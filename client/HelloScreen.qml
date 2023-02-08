import QtQuick 2.4
import xbot2_gui.common

HelloScreenForm {

    id: hello

    property ClientEndpoint client: undefined

    signal updateServerUrl(var host, var port)
    signal restartUi()

    function setError(msg) {
        msgText.text = "Error: " + msg
        msgText.color = CommonProperties.colors.err
    }

    function setConnected(msg) {
        msgText.text = "Status OK: " + msg
        msgText.color = CommonProperties.colors.ok
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

    resetUiBtn.onReleased: {
        console.log('Restart UI Pressed')
        restartUi()
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
}

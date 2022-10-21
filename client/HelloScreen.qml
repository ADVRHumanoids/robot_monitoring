import QtQuick 2.4

HelloScreenForm {

    id: hello

    property var client: undefined

    signal updateServerUrl(var host, var port)

    function setError(msg) {
        msgText.text = "Error: " + msg
        msgText.color = "red"
    }

    function setConnected(msg) {
        msgText.text = "Status OK: " + msg
        msgText.color = "green"
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
        updateServerUrl(serverHost, serverPort)
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

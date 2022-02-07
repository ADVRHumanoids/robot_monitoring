import QtQuick 2.4

HelloScreenForm {

    id: hello

    signal updateServerUrl(var host, var port)

    function setError(msg) {
        msgText.text = msg
        msgText.color = "red"
    }

    function setConnected(msg) {
        msgText.text = msg
        msgText.color = "green"
    }

    function setProgress(msg) {
        statusText.text = msg
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
}

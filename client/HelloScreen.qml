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

    Component.onCompleted: {

        // updated server url upon apply btn released
        applyBtn.released.connect(
                    function() {
                        updateServerUrl(serverHost, serverPort)
                    }
            )

        // dynamic margins

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

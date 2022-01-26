import QtQuick 2.0
import QtWebSockets 1.1



WebSocket {

    id: socket
    url: "ws://localhost:8080/ws"
    active: true

    signal jointStateReceived(var js)

    signal error(var msg)
    signal connected(var msg)

    onTextMessageReceived: function (message) {

        var obj = JSON.parse(message)

        if(obj.type === "joint_states")
        {
            jointStateReceived(obj)
        }
        else
        {
            console.log("unknown msg type " + obj.type + " received")
        }
    }

    onStatusChanged: function() {
        print("status changed [url = " + url + "]")
        if (socket.status === WebSocket.Error) {
            console.log("Error: " + socket.errorString)
            error(socket.errorString)
        } else if (socket.status === WebSocket.Open) {
            console.log("Server connected")
            connected("Connected to " + url)
        } else if (socket.status === WebSocket.Closed) {
            console.log("Socket closed")
        }
    }

}


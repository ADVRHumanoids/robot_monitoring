import QtQuick 2.0
import QtWebSockets
import "client.js" as Client

Item
{

    property var qmin: []
    property var qmax: []
    property var vmax: []
    property var taumax: []
    property var jointNames: []
    property string hostname: "localhost"
    property int port: 8080

    signal finalized()
    signal jointStateReceived(var js)
    signal error(var msg)
    signal connected(var msg)

    WebSocket {

        id: socket
        url: "ws://" + hostname + ":" + port + "/ws"
        active: true

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
                Client.httpRequest("http://" + hostname + ":" + port + "/info",
                                   onInfoReceived)
                connected("Connected to " + url)
            } else if (socket.status === WebSocket.Closed) {
                console.log("Socket closed")
            }
        }
    }

    function onInfoReceived(msg) {
        print('info received from server')
        qmin = msg.qmin
        qmax = msg.qmax
        vmax = msg.vmax
        taumax = msg.taumax
        jointNames = msg.jnames
        finalized()
    }

}

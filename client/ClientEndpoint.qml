import QtQuick 2.0
import QtWebSockets
import "client.js" as Client
import "sharedData.js" as SharedData

Item
{
    // note: the appData object is exposed by main.cpp
    // if running from web it contains the server address

    // server hostname
    property string hostname: appData.hostname

    // server port
    property int port: appData.port

    // alias for the underlying websocket's active property
    property alias active: socket.active

    // triggered by this object after the server configuration
    // has been received and saved to SharedData (see /info)
    signal finalized()

    // triggered upon reception of a new joint state msg
    signal jointStateReceived(var js)

    // triggered on socket error
    signal error(var msg)

    // triggered on socket successful connection
    signal connected(var msg)

    // triggerd upon reception of a proc msg
    signal procMessageReceived(var msg)
    
    // method for performing am http request
    function doRequest(verb, url, body, callback) {
        Client.httpRequest(verb,
                           "http://" + hostname + ":" + port + url,
                           body,
                           callback)
    }

    // websocket for streaming data
    WebSocket {

        id: socket
        url: "ws://" + hostname + ":" + port + "/ws"
        active: true

        onTextMessageReceived: function (message) {

            var obj = JSON.parse(message)

            if(obj.type === "joint_states")
            {
                SharedData.latestJointState = obj
                jointStateReceived(obj)
            }
            else if(obj.type === "proc")
            {
                procMessageReceived(obj)
            }
            else
            {
                console.log("unknown msg type " + obj.type + " received")
            }
        }

        onStatusChanged: {
            print("status changed [url = " + url + "]")
            if (socket.status === WebSocket.Error) {
                console.log("Error: " + socket.errorString)
                error(socket.errorString)
                active = false
            } else if (socket.status === WebSocket.Open) {
                console.log("Server connected")
                doRequest("GET", "/info", "", onInfoReceived)
                connected("Connected to " + url + ", requesting configuration..")
            } else if (socket.status === WebSocket.Closed) {
                console.log("Socket closed")
                active = false
            }
        }
    }

    function onInfoReceived(msg) {

        SharedData.processInfo = msg.proc_data

        if(!msg.success) {
            error('server replied: ' + msg.message)
            active = false
            return
        }

        connected('configuration received from server')
        SharedData.qmin = msg.qmin
        SharedData.qmax = msg.qmax
        SharedData.vmax = msg.vmax
        SharedData.taumax = msg.taumax
        SharedData.jointNames = msg.jnames
        SharedData.latestJointState = msg.jstate
        finalized()
    }

}

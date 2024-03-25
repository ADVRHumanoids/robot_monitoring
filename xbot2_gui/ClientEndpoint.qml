import QtQuick
import QtWebSockets
import QtCore
import QtQml.WorkerScript

import Common
import Network
import "ClientEndpoint.js" as Client
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

    //
    property bool isConnected: false

    // bool flag to indicate if we managed to receive all the
    // info about the running system (urdf data, plugin names, etc)
    property bool isFinalized: false

    // triggered by this object after the server configuration
    // has been received and saved to SharedData (see /info)
    signal finalized()

    // triggered upon reception of a new joint state msg
    signal jointStateReceived(var js)

    // receiving joint states
    property bool robotConnected: false

    // triggered on socket error
    signal error(var msg)

    // triggered on socket successful connection
    signal connected(var msg)

    // triggerd upon reception of a proc msg
    signal procMessageReceived(var msg)

    // triggerd upon reception of a plugin stat msg
    signal pluginStatMessageReceived(var msg)

    // image received
    signal jpegReceived(var msg)
    signal theoraPacketReceived(var msg)

    // generic message
    signal objectReceived(var msg)

    // bytes received counter
    property int bytesRecv: 0
    property int bytesSent: 0
    property real srvRtt: 0
    property int jsMsgRecv: 0
    property int jsDropped: 0


    // method for performing an http request
    function doRequest(verb, url, body, callback) {
        Client.httpRequest(verb,
                           "http://" + hostname + ":" + port + url,
                           body,
                           callback)
    }

    function doRequestRaw(verb, url, body, callback) {
        Client.httpRequestRaw(verb,
                           "http://" + hostname + ":" + port + url,
                           body,
                           callback)
    }

    function doRequestAsync(verb, url, body) {
        return Client.httpRequestAsync(verb,
                                       "http://" + hostname + ":" + port + url,
                                       body)
    }

    // method for sending a text message over websocket
    function sendTextMessage(msg) {
        if(socket.active) {
            bytesSent += msg.length
            socket.sendTextMessage(msg)
        }
    }

    // method for sending a text message over udp
    function sendTextMessageUdp(msg) {
        if(udp.bound) {
            bytesSent += msg.length
            udp.sendTextMessage(msg)
        }
    }


    // private
    id: root


    // websocket for streaming data
    WebSocket {

        id: socket
        url: "ws://" + hostname + ":" + port + "/ws"
        active: true

        onTextMessageReceived: function (message) {

            root.bytesRecv += message.length

            // send to worker thread for deserialization
            worker.sendMessage(message)

        }

        onStatusChanged: {

            print(`status changed [url ${url}]: ${socket.status}`)

            if (socket.status === WebSocket.Error) {
                CommonProperties.notifications.error('Error: ' + socket.errorString, 'webclient')
                error(socket.errorString)
                active = false
                isConnected = false
                root.isFinalized = false
            } else if (socket.status === WebSocket.Open) {
                CommonProperties.notifications.info('Server connected', 'webclient')
                connected('Server connected')
                isConnected = true
                root.bytesRecv = 0
                root.bytesSent = 0
                doRequestAsync("GET", "/udp", "")
                    .then((response) => {
                                  udp.hostname = root.hostname
                                  udp.port = response.port
                              })
            } else if (socket.status === WebSocket.Closed) {
                CommonProperties.notifications.error('Socket closed', 'webclient')
                isConnected = false
                active = false
                root.isFinalized = false
            }
        }
    }

    // udp socket to receive unreliable data
    UdpSocket {
        id: udp
        onTextMessageReceived: function (message) {
            root.bytesRecv += message.length
            worker.sendMessage(message)
        }
    }

    WorkerScript {

        id: worker
        source: "DeserializationWorker.mjs"
        onMessage: function(msg) {
            Client.handleMessage(msg)
        }
    }

    property int _nattempt: 0

    function onInfoReceived(msg) {

        _nattempt++

        if(!msg.success) {
            return
        }

        connected('configuration received from server')
        SharedData.qmin = msg.qmin
        SharedData.qmax = msg.qmax
        SharedData.vmax = msg.vmax
        SharedData.taumax = msg.taumax
        SharedData.jointNames = msg.jnames
        SharedData.latestJointState = msg.jstate
        isFinalized = true
        finalized()
    }


    Timer {
        id: pingTimer
        running: root.isConnected
        repeat: true
        interval: 300

        onTriggered: {
            let msg = Object()
            msg.type = 'ping'
            msg.cli_time_ns = appData.getTimeNs()
            root.sendTextMessageUdp(JSON.stringify(msg))
        }
    }

    Timer {
        id: robotConnectedTimer
        interval: 1000
        onTriggered: {
            root.robotConnected = false
        }
    }

    Timer  {
        id: broadcastUdpTimer
        interval: 1000
        repeat: true
        running: udp.bound
        onTriggered: {
            udp.sendTextMessage('udp_discovery')
        }
    }

    Settings {
        category: 'client'
        property alias hostname: root.hostname
        property alias port: root.port
    }

}

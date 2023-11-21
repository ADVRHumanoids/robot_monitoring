import QtQuick
import QtWebSockets
import QtCore

import Common
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


    // private
    id: root


    // websocket for streaming data
    WebSocket {

        id: socket
        url: "ws://" + hostname + ":" + port + "/ws"
        active: true

        onTextMessageReceived: function (message) {

            root.bytesRecv += message.length

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
            else if(obj.type === "jpeg")
            {
                jpegReceived(obj)
            }
            else if(obj.type === "theora")
            {
                theoraPacketReceived(obj)
            }
            else if(obj.type === "plugin_stats")
            {
                pluginStatMessageReceived(obj)
            }
            else if(obj.type === "heartbeat")
            {
                // do nothing
            }
            else if(obj.type === 'ping')
            {
                root.srvRtt = (appData.getTimeNs() - obj.cli_time_ns)*1e-6
            }
            else
            {
                objectReceived(obj)
            }
        }

        onStatusChanged: {
            print("status changed [url = " + url + "]")
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
            } else if (socket.status === WebSocket.Closed) {
                CommonProperties.notifications.error('Socket closed', 'webclient')
                isConnected = false
                active = false
                root.isFinalized = false
            }
        }
    }

    property int _nattempt: 0

    function onInfoReceived(msg) {

        _nattempt++

        if(!msg.success) {
            error('[' + _nattempt + '] server replied: ' + msg.message)
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

    // if websocket connection is up and running,
    // and we did not manage to finalize yet,
    // retry every 1 sec
    Timer {
        id: fetchInfoTimer
        interval: 1000
        repeat: true
        running: !root.isFinalized && root.isConnected
        triggeredOnStart: true
        property int _nattempt: 0

        onTriggered: {
            parent.connected("[" + _nattempt + "] connected to " + socket.url + ", requesting configuration..")
            doRequestAsync("GET", "/joint_states/info", "")
                .then((response) => {root.onInfoReceived(response)})
            _nattempt++
        }
    }

    Timer {
        id: pingTimer
        running: socket.active
        repeat: true
        interval: 300

        onTriggered: {
            let msg = Object()
            msg.type = 'ping'
            msg.cli_time_ns = appData.getTimeNs()
            sendTextMessage(JSON.stringify(msg))
        }
    }

    Settings {
        category: 'client'
        property alias hostname: root.hostname
        property alias port: root.port
    }

}

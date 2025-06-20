import QtQuick
import QtWebSockets
import QtCore

import Common
import Network
import Protobuf

import "ClientEndpoint.js" as Client
import "sharedData.js" as SharedData

import xbot2_gui.msgs

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

    // deserialization worker
    property alias worker: workerLoader.item

    //
    property bool isConnected: false

    // bool flag to indicate if we managed to receive all the
    // info about the running system (urdf data, plugin names, etc)
    property bool isFinalized: false

    // triggered by this object after the server configuration
    // has been received and saved to SharedData (see /info)
    signal finalized()

    // triggered upon reception of a new joint state msg
    signal jointStateReceived(jointState js)

    // receiving joint states
    property bool robotConnected: false

    // triggered on socket error
    signal error(var msg)

    // triggered on socket successful connection
    signal connected(var msg)

    // triggerd upon reception of a proc msg
    signal processOutputReceived(processOutput msg)

    // triggerd upon reception of a plugin stat msg
    signal pluginStatMessageReceived(var msg)

    // image received
    signal jpegReceived(var msg)
    signal theoraPacketReceived(theoraPacket msg)

    // generic message
    signal objectReceived(var msg)

    // bytes received counter
    property alias bytesRecvCounters: pb.recvBytes
    property alias numMsgCounters: pb.numMsg
    property int bytesRecv: pb.recvBytes.all
    property int bytesSent: 0
    property real srvRtt: 0
    property int jsMsgRecv: 0
    property int jsDropped: 0


    // method for performing an http request
    function doRequest(verb, url, body, callback, quiet = false) {
        Client.httpRequest(verb,
                           "http://" + hostname + ":" + port + url,
                           body,
                           callback,
                           quiet)
    }

    function doRequestRaw(verb, url, body, callback) {
        Client.httpRequestRaw(verb,
                           "http://" + hostname + ":" + port + url,
                           body,
                           callback)
    }

    function doRequestAsync(verb, url, body, quiet = false) {
        return Client.httpRequestAsync(verb,
                                       "http://" + hostname + ":" + port + url,
                                       body,
                                       quiet)
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

        if(appData.wasm) {
            sendTextMessage(msg)
        }

        if(udp.bound) {
            bytesSent += msg.length
            udp.sendTextMessage(msg)
        }
    }


    // private
    id: root


    // websocket for streaming data

    WebSocketAsync {

        id: socket
        url: "ws://" + hostname + ":" + port + "/ws"
        active: true

        onBinaryMessageReceived: function (data) {
            // root.bytesRecv += data.byteLength
            pb.processBinaryMessage(data)
        }

        onConnected: {

            CommonProperties.notifications.info('Server connected', 'webclient')

            root.connected('Server connected')

            root.isConnected = true

            // root.bytesRecv = 0
            root.bytesSent = 0

            if(appData.wasm) {
                root.sendTextMessage(
                            JSON.stringify(
                                {
                                    'type': 'request_ws_udp_tunnel'
                                }
                                )
                            )
            }
            else {
                root.doRequestAsync("GET", "/udp", "")
                .then((response) => {
                          udp.hostname = root.hostname
                          udp.port = response.port
                      })
            }
        }

        onDisconnected: {
            root.isConnected = false
            root.isFinalized = false
            udp.rebind()
        }

        onErrorOccurred: function(err) {
            CommonProperties.notifications.error('Error: ' + err, 'webclient')
            root.error(err)
        }
    }

    ProtobufDeserialization {

        id: pb

        onTextMessageReceived: function (message) {

            if(appData.wasm) {
                // deserialize directly since workers have issues in wasm
                Client.handleMessage(JSON.parse(message))
            }
            else {
                // send to worker thread for deserialization
                worker.sendMessage(message)
            }

        }

        onJointStateReceived: function(js) {

            root.robotConnected = true

            robotConnectedTimer.restart()

            SharedData.latestJointState = js

            SharedData1.latestJointState = js

            root.jointStateReceived(js)

            root.jsMsgRecv += 1

            // if(lastJsSeqId < 0) {
            //     lastJsSeqId = obj.seq
            // }
            // else {
            //     root.jsDropped += (obj.seq - lastJsSeqId - 1)
            //     lastJsSeqId = obj.seq
            // }

            if(isConnected && !isFinalized)
            {
                client.active = true

                doRequestAsync("GET", "/joint_states/info", "")
                        .then((response) => {
                              root.onInfoReceived(response)
                          })
            }
        }

        onProcessOutputReceived: function(po) {
            root.processOutputReceived(po)
        }

        onTheoraPacketReceived: function(pkt) {
            root.theoraPacketReceived(pkt)
        }

    }

    // udp socket to receive unreliable data
    UdpSocket {
        id: udp
        onBinaryMessageReceived: function (data) {
            // root.bytesRecv += data.byteLength
            pb.processBinaryMessage(data)
        }
    }

    Loader {

        source: "DeserializationWorker.qml"

        id: workerLoader

        active: !appData.wasm

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

    Timer {
        id: retryConnect
        interval: 2000
        repeat: true
        running: !root.active

        onTriggered: {
            doRequestAsync('GET', '/version', '', true)
            .then((res) => {
                   console.log('server is alive, connecting ws')
                   root.active = true
                  })
        }
    }

    Settings {
        category: 'client'
        property alias hostname: root.hostname
        property alias port: root.port
    }

    Component.onCompleted: {
        if(appData.portFromCmdLine) {
            root.port = appData.port
        }
    }

}

import QtQuick
import "ClientEndpoint.js" as Client

WorkerScript {

    id: worker
    source: "DeserializationWorker.js"
    onMessage: function(msg) {
        Client.handleMessage(msg)
    }
}

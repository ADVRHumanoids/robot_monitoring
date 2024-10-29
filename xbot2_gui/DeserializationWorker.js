WorkerScript.onMessage = function(message) {
    try
    {
        let obj = JSON.parse(message)
        WorkerScript.sendMessage(obj)
    }
    catch (error) {
        console.error('received invalid json (' + error + '): ' + message);
    }
}

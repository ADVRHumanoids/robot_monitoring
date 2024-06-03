WorkerScript.onMessage = function(message) {
    let obj = JSON.parse(message)
    WorkerScript.sendMessage(obj)
}

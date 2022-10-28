function sendVref() {
    console.log('sending ' + JSON.stringify(vref))
    var msg = {
        'type': 'velocity_command',
        'task_name': taskCombo.currentText,
        'vref': vref
    }
    client.sendTextMessage(JSON.stringify(msg))
}

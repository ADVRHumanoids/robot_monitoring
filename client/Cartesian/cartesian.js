function sendVref() {
    var msg = {
        'type': 'velocity_command',
        'task_name': taskCombo.currentText,
        'vref': vref
    }
    client.sendTextMessage(JSON.stringify(msg))
}

function updateTaskNames() {
    client.doRequest('GET', '/cartesian/get_task_list', '',
         function(response) {
             var cartesianTaskNames = []
             for(let i = 0; i < response.names.length; i++) {
                if(response.types[i] === "Cartesian") {
                    cartesianTaskNames.push(response.names[i])
                }
             }
             taskCombo.model = cartesianTaskNames
         })
}

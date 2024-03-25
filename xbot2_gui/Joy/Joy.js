function sendVref(task, vref) {
    var msg = {
        'type': 'velocity_command',
        'task_name': task,
        'vref': vref
    }
    client.sendTextMessageUdp(JSON.stringify(msg))
    vref = [0, 0, 0, 0, 0, 0]
}


function enableTask(task_name, callback) {

    if(task_name.startsWith('[simple topic]'))
    {
        callback()
        return
    }

    let req_body = Object()
    req_body.task_name = task_name
    req_body.control_mode = 'velocity'

    let req_callback = function(response) {
        if(!response.success) {
            console.log('error: ' + response.message)
        }
        else {
            callback()
        }
    }

    client.doRequest('PUT',
                     '/cartesian/' + task_name + '/set_control_mode',
                     JSON.stringify(req_body),
                     req_callback)
}


function disableTask(task_name, callback) {

    if(task_name.startsWith('[simple topic]'))
    {
        callback()
        return
    }

    let req_body = Object()
    req_body.task_name = task_name
    req_body.control_mode = 'position'

    let req_callback = function(response) {
        if(!response.success) {
            console.log('error: ' + response.message)
        }
        else {
            callback()
        }
    }

    client.doRequest('PUT',
                     '/cartesian/' + task_name + '/set_control_mode',
                     JSON.stringify(req_body),
                     req_callback)
}


function taskIsEnabled(task_name, callback) {

    if(task_name.startsWith('[simple topic]'))
    {
        callback(true)
        return
    }

    let req_callback = function(response) {
        if(!response.success) {
            console.log('error: ' + response.message)
        }
        else {
            console.log('got response: ' + JSON.stringify(response))
            callback(response.control_mode === 'Velocity')
        }
    }

    client.doRequest('GET',
                     '/cartesian/' + task_name + '/get_cartesian_task_properties',
                     '',
                     req_callback)
}


function updateTaskNames(taskCombo) {
    client.doRequest('GET', '/cartesian/get_task_list', '',
                     function(response) {

                         if(!response.success) {
                             ikRunning = false
                             return
                         }

                         ikRunning = true

                         var cartesianTaskNames = []
                         for(let i = 0; i < response.names.length; i++) {
                             console.log(`got task ${response.types[i]} ${response.names[i]}`)
                             if(response.types[i] === "Cartesian") {
                                 cartesianTaskNames.push(response.names[i])
                             }
                             if(response.types[i] === "SimpleTopic") {
                                 cartesianTaskNames.push('[simple topic] '+response.names[i])
                             }
                         }
                         taskCombo.model = cartesianTaskNames
                     })
}

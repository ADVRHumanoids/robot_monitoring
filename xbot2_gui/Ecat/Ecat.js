function construct() {

    client.doRequest('GET', '/joint_states/info', '',
                     function (msg) {
                        jointNames = msg.jnames
                     })

}

function stop(jname) {

    client.doRequestAsync('POST', `/ecat/stop_motor?name=${jname}`, '')
        .then((msg) => {
                            console.log(msg.message)
                         })

}

let ctrlMap = {
    'Position': 'posmode',
    'Impedance': 'impmode',
    'Velocity': 'velmode',
}

function start(jname, ctrl) {

    client.doRequestAsync('POST', `/ecat/start_motor?name=${jname}&ctrl=${ctrlMap[ctrl]}`, '')
        .then((msg) => {
                            console.log(msg.message)
                         })

}

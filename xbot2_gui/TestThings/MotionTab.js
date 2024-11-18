function stop() {
    statusText.text = 'Stopping motor'
    client.doRequestAsync('POST', `/dashboard/robot_switch/stop`, '')
        .then(function(res) {

            if(!res.success)
            {
                stateConnectionError.reason = 'Stop failed: ' + res.message
                root.connectionError()
            }

            stopped()
        })
}

function connect() {

    let params = {
        'motor_type': cfg.selectedMotorType,
        'load_mass': cfg.loadMass,
        'load_radius': cfg.loadRadius
    }
    statusText.text = 'Starting motor'
    client.doRequestAsync('POST', `/dashboard/robot_switch/start`, JSON.stringify({'params': params}))
        .then(function(res) {
            if(!res.success)
            {
                stateConnectionError.reason = 'Start failed: ' + res.message
                root.connectionError()
            }

            connected()
        })
}


function startAcquisition(trj) {

    // statusText.text = 'Configuring data acquisition...'

    let params = {
        'motor_type': cfg.selectedMotorType,
        'load_mass': cfg.loadMass,
        'load_radius': cfg.loadRadius,
        'trj': trj
    }

    client.doRequestAsync('POST',
                          '/hhcm_calibration/configure',
                          JSON.stringify(params))
        .then(function(res){
            if(!res.success)
            {
                statusText.text = 'Configuration error'
                return
            }

            root.calibDataDir = res.data_dir

            return client.doRequestAsync('POST', '/hhcm_calibration/start')
        })
}

function stopTrajectory() {
    client.doRequest('PUT', '/plugin/trajectory/command/stop', '',
                     (m) => {
                         if(m.success)
                         {
                             trjProgress = -1
                         }
                     })
}

function calibrate() {

    // statusText.text = 'Configuring data acquisition...'

    let body = {
        'data_dir': root.calibDataDir
    }

    console.log('requesting calib...')

    client.doRequestAsync('POST',
                          '/hhcm_calibration/calibrate',
                          JSON.stringify(body))
        .then(function(res){
            if(!res.success)
            {
                calibOutputText.text = 'Calibration error \n' + res.stderr
                return
            }

            console.log('...OK')
            calibOutputText.text = res.calib_result
        })
}

function upload() {

    // statusText.text = 'Configuring data acquisition...'

    let body = {
    }

    console.log('requesting calib...')

    client.doRequestAsync('POST',
                          '/hhcm_calibration/upload',
                          JSON.stringify(body))
        .then(function(res){
            if(!res.success)
            {
                calibOutputText.text = 'Upload error \n' + res.stderr
                return
            }

            console.log('...OK')
            calibOutputText.text = 'Started syncing with onedrive client...'
        })
}

function updateMotorProperties() {

    let callback = function (obj) {

        console.log(JSON.stringify(obj))

        motorProperties = obj.data

        let motor_types = []

        for (const [key, value] of Object.entries(motorProperties)) {
            motor_types.push(key)
        }

        motorCombo.model = motor_types

    }

    client.doRequest('GET',
                     '/hhcm_calibration/properties',
                     '',
                     callback)
}


function construct() {

    let props = Object()

    props.type = 'joint_state'
    props.jIndex = 0
    props.jName = 'j_motor'

    props.fieldName = 'linkPos'
    positionPlot.addSeries(props.fieldName, props)

    props.fieldName = 'motPos'
    positionPlot.addSeries(props.fieldName, props)

    props.fieldName = 'posRef'
    positionPlot.addSeries(props.fieldName, props)

    props.fieldName = 'linkVel'
    velocityPlot.addSeries(props.fieldName, props)

    props.fieldName = 'velRef'
    velocityPlot.addSeries(props.fieldName, props)

    props.fieldName = 'motVel'
    velocityPlot.addSeries(props.fieldName, props)

    props.fieldName = 'tor'
    torquePlot.addSeries(props.fieldName, props)

    props.fieldName = 'torRef'
    torquePlot.addSeries(props.fieldName, props)

    props.fieldName = 'motTor'
    torquePlot.addSeries(props.fieldName, props)

    props.fieldName = 'fc'
    frictionPlot.addSeries(props.fieldName, props)

    console.log('construct done')

}


function jsCallback(msg) {

    // update plots

    if(initialTime < 0) {
        initialTime = msg.stamp
    }

    let t = msg.stamp - initialTime

    positionPlot.addPointsFromMsg(t, msg)

    velocityPlot.addPointsFromMsg(t, msg)

    torquePlot.addPointsFromMsg(t, msg)

    frictionPlot.addPoint('fc', msg.motVel[0], msg.tor[0] - msg.motTor[0])

    // torquePlot.addPoint(torquePlot.refSeries, t, msg.motTor[0])

    // updated 3d viewer

    // robotViewer.updateRobotState(msg,
    //                              robotViewer.robotState,
    //                              'linkPos')
}

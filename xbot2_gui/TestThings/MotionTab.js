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
        .catch(function(error) {
            stateConnectionError.reason = error
            root.connectionError()
        })
}

function connect() {

    let params = {
        'motor_type': 'small_30',
        'load_mass': parseFloat(loadMassText.text),
        'load_radius': parseFloat(loadRadiusText.text)*0.01
    }

    statusText.text = 'Starting motor'
    client.doRequestAsync('POST', `/dashboard/robot_switch/start`, JSON.stringify({'params': params}))
        .then(function(res) {
            if(!res.success)
            {
                stateConnectionError.reason = 'Start failed: ' + res.message
                root.connectionError()
            }

            return client.doRequestAsync('GET', `/hhcm_calibration/properties`, '')
        })
        .then(function(res) {
            if(!res.success)
            {
                stateConnectionError.reason = 'Failed to get motor properties: ' + res.message
                root.connectionError()
            }

            motorProperties = res.data[res.motor_type]
            motorType = res.motor_type
            motorId = res.motor_id

            if(motorType === '' || motorId === '') {
                stateConnectionError.reason = `Invalid motor ID "${motorId}" or type "${motorType}"`
                root.connectionError()
            }

            connected()
        }
        )
        .catch(function(error) {
            stateConnectionError.reason = error
            root.connectionError()
        })
}


function startAcquisition(trj) {

    // statusText.text = 'Configuring data acquisition...'

    let params = {
        'motor_id': root.motorId,
        'motor_type': root.motorType,
        'load_mass': parseFloat(loadMassText.text),
        'load_radius': parseFloat(loadRadiusText.text)*0.01,
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

function getCalibList() {
    client.doRequestAsync('GET',
                          '/hhcm_calibration/get_data_dirs')
    .then(function(res){
        calibCombo.model = res.result
    })

}

function loadCalibResult(dataDir) {

    // statusText.text = 'Configuring data acquisition...'

    let body = {
        'data_dir': dataDir
    }

    client.doRequestAsync('POST',
                          '/hhcm_calibration/load_calib_result',
                          JSON.stringify(body))
        .then(function(res){
            if(!res.success)
            {
                calibOutputText.text = 'Calibration error \n' + res.stderr
                return
            }

            let calibDataPlot = calibDataPlotLoader.item

            calibDataPlot.addSeries('tau_mot', {})
            calibDataPlot.addSeries('tau_mot_ls_estimate', {})

            console.log(`adding npoints = ${res.tau_mot.length}....`)

            calibDataPlot.setPoints('tau_mot', 0.001, res.tau_mot)
            calibDataPlot.setPoints('tau_mot_ls_estimate', 0.001, res.tau_mot_ls_estimate)

            console.log('....done')

            calibDataPlot.setXRange(0, res.tau_mot.length*0.001)
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

        motorTypeText.text = obj.motor_type
        motorIdText.text = obj.motor_id
        scanBusy.visible = false

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

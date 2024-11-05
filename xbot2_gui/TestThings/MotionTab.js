function connect() {

    // TODO deploy
    statusText.text = 'Stopping...'
    client.doRequestAsync('POST', `/dashboard/robot_switch/stop`, '')
        .then(function(res) {

            if(!res.success)
            {
                statusText.text = 'Stop failed: ' + res.message
                throw new Error(res.message);
            }

            viewerLoader.active = false

            let params = {
                'motor_type': cfg.selectedMotorType,
                'load_mass': cfg.loadMass,
                'load_radius': cfg.loadRadius
            }

            statusText.text = 'Connecting...'
            return client.doRequestAsync('POST', `/dashboard/robot_switch/start`, JSON.stringify({'params': params}))
        })
        .then(function(res) {
            statusText.text = res.success ? 'Connected' : 'Connection failed'
            viewerLoader.active = true
        })
}


function startAcquisition() {

    statusText.text = 'Configuring data acquisition...'

    let params = {
        'log_file': `${cfg.selectedMotorType}_${cfg.lockedOutput ? "LOCKED" : "FREE"}_${cfg.loadMass.toFixed(2)}KG_${cfg.loadRadius.toFixed(2)}R`,
        'freq_min': cfg.selectedMotorProperties.trajectory_freq_min,
        'freq_max': cfg.selectedMotorProperties.trajectory_freq_max
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

            statusText.text = Qt.binding(() => {return 'Trajectory: ' + root.trjPluginState})
            return client.doRequestAsync('PUT', '/plugin/trajectory/command/start')
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
    positionPlot.linkSeries = positionPlot.addSeries(props.jName + '_' + props.fieldName, props, false)

    props.fieldName = 'motPos'
    positionPlot.refSeries = positionPlot.addSeries(props.jName + '_' + props.fieldName, props, false)

    props.fieldName = 'posRef'
    positionPlot.motSeries = positionPlot.addSeries(props.jName + '_' + props.fieldName, props, false)

    props.fieldName = 'linkVel'
    velocityPlot.linkSeries = velocityPlot.addSeries(props.jName + '_' + props.fieldName, props, false)

    props.fieldName = 'velRef'
    velocityPlot.refSeries = velocityPlot.addSeries(props.jName + '_' + props.fieldName, props, false)

    props.fieldName = 'motVel'
    velocityPlot.motSeries = velocityPlot.addSeries(props.jName + '_' + props.fieldName, props, false)

    console.log('construct done')

}


function jsCallback(msg) {

    // update plots

    if(initialTime < 0) {
        initialTime = msg.stamp
    }

    let t = msg.stamp - initialTime

    positionPlot.addPoint(positionPlot.linkSeries, t, msg.linkPos[0])

    positionPlot.addPoint(positionPlot.motSeries, t, msg.motPos[0])

    positionPlot.addPoint(positionPlot.refSeries, t, msg.posRef[0])

    velocityPlot.addPoint(velocityPlot.linkSeries, t, msg.linkVel[0])

    velocityPlot.addPoint(velocityPlot.motSeries, t, msg.motVel[0])

    velocityPlot.addPoint(velocityPlot.refSeries, t, msg.velRef[0])

    // updated 3d viewer

    robotViewer.updateRobotState(msg,
                                 robotViewer.robotState,
                                 'linkPos')
}

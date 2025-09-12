.import "/qt/qml/Main/sharedData.js" as SharedData
.import Common 1.0 as Common
let error = Common.CommonProperties.notifications.error

function handleFault(jName, faultCode) {
    barPlot.setStatus(jName, faultCode === "" ? true : false)
    jointState.setFaultCode(jName, faultCode)
}

let vbattCurrentThreshold = 49.5

function jsCallback(js) {

    if(root.isCurrentPage) {
        robotViewer.updateRobotState(js,
                                     robotViewer.robotState,
                                     'linkPos')

        barPlot.setJointStateMessage(js)

        jointState.setJointStateMessage(js)
    }

    addJointStatePoint(livePlot, js)

    for(let aux of Object.keys(js.aux)) {
        barPlot.addAuxType(aux)
    }

    if(js.vbatt > 0 && js.vbatt < vbattCurrentThreshold) {
        error(`battery voltage V_batt =  ${js.vbatt} V`, 'PMS')
        vbattCurrentThreshold = js.vbatt - 0.5
    }

    if(js.vbatt > vbattCurrentThreshold + 2) {
        vbattCurrentThreshold = js.vbatt - 0.5
    }

    vbatt = js.vbatt
    iload = js.ibatt
}


function objCallback(obj) {

    if(obj.type === 'joint_device_info') {

        jointDevice.filterActive = obj.filter_active

        jointDevice.filterCutoff = obj.filter_cutoff_hz

        if(jointDevice.jointActive && !obj.joint_active) {
            error('Safety triggered', 'robot')
        }

        jointDevice.jointActive = obj.joint_active

    }
    else if(obj.type === 'joint_fault') {
        let names = obj.name
        for(let i = 0; i < names.length; i++) {
            handleFault(names[i], obj.fault[i])
        }
    }
}


function addJointStateSeries(livePlot, jName, fieldName) {

    if(livePlot === null) {
        error('Live plot unavailable: open the "Plot" panel', 'Plot')
        return
    }

    let seriesName = jName + '/' + fieldName

    let props = Object()
    props.type = 'joint_state'
    props.jIndex = 0
    props.jName = jName
    props.fieldName = fieldName

    livePlot.addSeries(seriesName, props)

}

function addJointStatePoint(livePlot, msg) {

    if(livePlot === null) {
        return
    }

    // iterate over current series
    for(let [seriesName, seriesData] of Object.entries(livePlot.currSeries)) {

        // get series properties
        let props = seriesData.properties

        // is it a joint series?
        if(props.type !== 'joint_state') {
            return
        }

        // recompute jIndex if needed
        if(SharedData.jointNames[props.jIndex] !== props.jName) {
            props.jIndex = SharedData.jointNames.indexOf(props.jName)
        }

        // compute relative time
        let t = msg.stamp ?? appData.getTimeNs() * 1e-9

        if(livePlot.initialTime < 0) {
            livePlot.initialTime = t
        }

        t = t - livePlot.initialTime

        // get value from msg
        let val = msg[props.fieldName][props.jIndex]

        // add point to plot
        livePlot.addPoint(seriesData, t, val)

    }
}


function setFilterActive(active) {

    client.doRequest('POST', '/joint/safety/set_filter_active?active=' + active,
                     '',
                     (msg) =>
                     {
                         if(!msg.success) {
                             console.error(msg.message)
                         }
                     })
}

function setFilterProfile(profile) {

    client.doRequest('POST', '/joint/safety/set_filter_profile?profile=' + profile,
                     '',
                     (msg) =>
                     {
                         if(!msg.success) {
                             console.error(msg.message)
                         }
                     })
}

function setSafetyState(ok, cli=undefined) {

    let client = cli ? cli : root.client

    client.doRequest('POST', '/joint/safety/set_enabled?enabled=' + ok,
                     '',
                     (msg) =>
                     {
                         if(!msg.success) {
                             console.error(msg.message)
                         }
                     })
}

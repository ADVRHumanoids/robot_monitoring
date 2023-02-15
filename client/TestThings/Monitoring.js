function handleFault(jName, faultCode) {
    barPlot.setStatus(jName, faultCode === "" ? true : false)
    jointState.setFaultCode(jName, faultCode)
}

function jsCallback(js) {
    barPlot.setJointStateMessage(js)
    jointState.setJointStateMessage(js)
    livePlot.setJointStateMessage(js)
}

function objCallback(obj) {

    if(obj.type === 'joint_device_info') {
        jointDevice.filterActive = obj.filter_active
        jointDevice.filterCutoff = obj.filter_cutoff_hz
        jointDevice.jointActive = obj.joint_active
    }
    else if(obj.type === 'joint_fault') {
        console.log(JSON.stringify(obj))
        let names = obj.name
        for(let i = 0; i < names.length; i++) {
            handleFault(names[i], obj.fault[i])
        }
    }
}

function construct() {
    client.jointStateReceived.connect(jsCallback)
    client.objectReceived.connect(objCallback)
}

function destroy() {
    client.jointStateReceived.disconnect(jsCallback)
    client.objectReceived.disconnect(objCallback)
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

function setSafetyState(ok) {
    client.doRequest('POST', '/joint/safety/set_enabled?enabled=' + ok,
                     '',
                     (msg) =>
                     {
                         if(!msg.success) {
                             console.error(msg.message)
                         }
                     })
}

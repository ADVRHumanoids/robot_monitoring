.import "/qt/qml/Main/sharedData.js" as SharedData

let cmdFields = ['posRef', 'k', 'd']
let cmdFieldsLong = ['Position', 'Stiffness', 'Damping']

let cmdFieldsShortToLong = {
    'posRef': 'Position',
    'k': 'Stiffness',
    'd': 'Damping'
}

function sendCommand(ctrlJoints, cmdField, ref, trjtime) {

    let jointNames = ctrlJoints.join(';')
    let cmd = (Array(ctrlJoints.length).fill(ref)).join(';')

    client.doRequestAsync('PUT',
                          `/joint_command/goto/${jointNames}?qref=${cmd}&time=${trjtime}&ctrl=${cmdField}`)
    .then((response) => trjCmdBtn.running = false)
    .catch((err) => console.error(err))

}

function stopCommand(jointName, cmd) {

    client.doRequestAsync('POST',
                          `/joint_command/goto/stop`)
    .then((response) => trjCmdBtn.running = false)
    .catch((err) => console.error(err))

}

function sliderRange(ctrlJoints, cmdField) {

    if(ctrlJoints.length === 0) {
        return [0, 0]
    }

    let idx = ctrlJoints.map(n => SharedData.jointNames.indexOf(n))

    if(cmdField === 'Position') {
        let qmin = idx.map(i => SharedData.qmin[i])
        let qmax = idx.map(i => SharedData.qmax[i])
        qmin = Math.max(...qmin)
        qmax = Math.min(...qmax)
        console.log(`Range for ${ctrlJoints} is ${qmin} ${qmax}`)
        return [qmin, qmax]
    }

    if(cmdField === 'Stiffness') {
        let k = idx.map(i => SharedData.latestJointState.k[i])
        let kmax = Math.min(...k) * 10.0
        return [0, kmax]
    }

    if(cmdField === 'Damping') {
        let d = idx.map(i => SharedData.latestJointState.d[i])
        let dmax = Math.min(...d) * 10.0
        return [0, dmax]
    }
}

function updateQ(q, ctrlJoints) {

    for(let i = 0; i < ctrlJoints.length; i++) {
        let j = ctrlJoints[i]
        q[robotCmd.jointNames.indexOf(j)] = slider.value
    }

    return q
}

function currentValue(ctrlJoints, cmdField) {

    let idx = SharedData.jointNames.indexOf(ctrlJoints[0])

    if(cmdField === 'Position') {
        return SharedData.latestJointState.posRef[idx]
    }

    if(cmdField === 'Stiffness') {
        return SharedData.latestJointState.k[idx]
    }

    if(cmdField === 'Damping') {
        return SharedData.latestJointState.d[idx]
    }
}

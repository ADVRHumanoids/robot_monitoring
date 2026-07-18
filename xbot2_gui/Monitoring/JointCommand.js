.import "/qt/qml/Main/sharedData.js" as SharedData

let cmdFieldsLong = ['Position', 'Velocity', 'Effort', 'Stiffness', 'Damping']

function sendCommand(ctrlJoints, cmdField, ref, trjtime) {

    let jointNames = ctrlJoints.join(';')
    let cmd = (Array(ctrlJoints.length).fill(ref)).join(';')

    client.doRequestAsync('PUT',
                          `/joint_command/goto/${jointNames}?qref=${cmd}&time=${trjtime}&ctrl=${cmdField}`)
    .then((response) => {
              trjCmdBtn.running = false
              robotViewer.showRobotCmd = false
          })
    .catch((err) => console.error(err))

}

function sendContinuousCommand(ctrlJoints, cmdField, ref) {

    let msg = Object()
    msg.type = 'joint_cmd'
    msg.joint_names = ctrlJoints
    msg.command = Array(ctrlJoints.length).fill(ref)
    msg.ctrl = cmdField

    client.sendTextMessageUdp(JSON.stringify(msg))

    console.log(JSON.stringify(msg))

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
        return [qmin, qmax]
    }

    if(cmdField === 'Velocity') {
        let vmax = idx.map(i => SharedData.vmax[i])
        vmax = Math.min(...vmax)
        return [-vmax, vmax]
    }

    if(cmdField === 'Effort') {
        let taumax = idx.map(i => SharedData.taumax[i])
        taumax = Math.min(...taumax)
        taumax = Math.min(taumax, 50.0)
        return [-taumax, taumax]
    }

    if(cmdField === 'Stiffness') {
        let k = idx.map(i => SharedData.latestJointState.k[i])
        let kmax = Math.max(Math.min(...k) * 10.0, 100.0)
        return [0, kmax]
    }

    if(cmdField === 'Damping') {
        let d = idx.map(i => SharedData.latestJointState.d[i])
        let dmax = Math.max(Math.min(...d) * 10.0, 50.0)
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

    if(cmdField === 'Velocity') {
        return 0.0
    }

    if(cmdField === 'Effort') {
        return 0.0
    }

    if(cmdField === 'Stiffness') {
        return SharedData.latestJointState.k[idx]
    }

    if(cmdField === 'Damping') {
        return SharedData.latestJointState.d[idx]
    }
}

function currentRef(ctrlJoints, activeCtrl) {

    let idx = SharedData.jointNames.indexOf(ctrlJoints[0])

    if(activeCtrl === 'Position') {
        return SharedData.latestJointState.posRef[idx]
    }

    if(activeCtrl === 'Velocity') {
        return SharedData.latestJointState.velRef[idx]
    }

    if(activeCtrl === 'Effort') {
        return SharedData.latestJointState.torRef[idx]
    }

    if(activeCtrl === 'Stiffness') {
        return SharedData.latestJointState.k[idx]
    }

    if(activeCtrl === 'Damping') {
        return SharedData.latestJointState.d[idx]
    }
}

function updateGripperNames() {
    client.doRequestAsync('GET',
                          `/joint_states/grippers`)
        .then((response) => gripperCombo.model = response.gripper_names)
        .catch((err) => console.error(err))
}

function sendGripperCommand(name, action, effort=0) {
    let msg = Object()
    msg.type = 'gripper_cmd'
    msg.name = name
    msg.action = action
    msg.effort = effort

    client.sendTextMessage(JSON.stringify(msg))

    console.log(JSON.stringify(msg))
}

function startMotor(name) {
    let jointNames = ctrlJoints.join(';')
    client.doRequestAsync('POST',
                          `/joint_command/motor_ctrl/${jointNames}?ctrl=start`)
        .then((response) => {

              })
}

function stopMotor(name) {
    let jointNames = ctrlJoints.join(';')
    client.doRequestAsync('POST',
                          `/joint_command/motor_ctrl/${jointNames}?ctrl=stop`)
        .then((response) => {

              })

}

function engageBrake(name) {
    let jointNames = ctrlJoints.join(';')
    client.doRequestAsync('POST',
                          `/joint_command/brake_ctrl/${jointNames}?ctrl=engage`)
        .then((response) => {

              })

}

function releaseBrake(name) {
    let jointNames = ctrlJoints.join(';')
    client.doRequestAsync('POST',
                          `/joint_command/brake_ctrl/${jointNames}?ctrl=release`)
        .then((response) => {

              })

}


function enableControl() {
    client.doRequest('PUT',
                     '/plugin/' + rosCtrlName + '/command/start',
                     '',
                     function(msg){console.log(JSON.stringify(msg))})
}

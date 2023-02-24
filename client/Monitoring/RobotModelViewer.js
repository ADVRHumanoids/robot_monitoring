function updateViewerState(js, robot, fieldName) {

    if(robot.ndof === 0) {
        return
    }

    robot.q = updateViewerQ(js,
                            robot.jointNames,
                            fieldName,
                            [...robot.q])

}


function updateViewerQ(js, jointNames, fieldName, q) {

    for(let i = 0; i < js.name.length; i++) {

        let name = js.name[i]
        let idx = jointNames.indexOf(name)

        q[idx] = js[fieldName][i]

    }

    return q
}


function sendCommand(jointName, cmd) {

    client.doRequestAsync('PUT',
                          `/joint_command/goto/${jointName}?qref=${cmd}&time=${3}`)
    .then((response) => trjCmdBtn.running = false)
    .catch((err) => console.error(err))

}

function stopCommand(jointName, cmd) {

    client.doRequestAsync('POST',
                          `/joint_command/goto/stop`)
    .then((response) => trjCmdBtn.running = false)
    .catch((err) => console.error(err))

}

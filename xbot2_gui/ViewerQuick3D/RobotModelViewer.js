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


    console.log(`updateViewerQ1 ${JSON.stringify(q)}`)

    for(let i = 0; i < js.name.length; i++) {

        let name = js.name[i]
        let idx = jointNames.indexOf(name)
        if(idx < 0)
        {
            continue
        }

        console.log(`${q[idx]} ${js[fieldName][i]}`)

        q[idx] = js[fieldName][i]

    }


    console.log(`updateViewerQ2 ${JSON.stringify(q)}`)

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

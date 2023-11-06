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

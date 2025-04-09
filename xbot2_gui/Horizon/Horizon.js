function sendVref(vref) {
    var msg = {
        'type': 'horizon_vref',
        'vref': vref
    }
    console.log(JSON.stringify(vref))
    client.sendTextMessage(JSON.stringify(msg))
    vref = [0, 0, 0, 0, 0, 0]
}

function walkSwitch(active) {
    return client.doRequestAsync('POST',
                          `/horizon/${configPane.gaitType.toLowerCase()}/switch?active=${active}`, '')
    .then((res) => {
              console.log(res.message)

          })
    .catch((err) => {})
}

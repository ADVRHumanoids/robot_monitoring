function sendVref(vref) {
    var msg = {
        'type': 'horizon_vref',
        'vref': vref
    }
    client.sendTextMessage(JSON.stringify(msg))
    vref = [0, 0, 0, 0, 0, 0]
}

function walkSwitch(active) {
    return client.doRequestAsync('POST',
                          `/horizon/walk/switch?active=${active}`, '')
    .then((res) => {
              console.log(res.message)

          })
    .catch((err) => {})
}

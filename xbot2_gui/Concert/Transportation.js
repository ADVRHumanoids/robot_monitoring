.import Common 1.0 as Common
.import Audio 1.0 as Audio
let error = Common.CommonProperties.notifications.error

function setAsrState(active) {
    client.doRequestAsync('POST', `/speech/enable_commands?active=${active ? 1 : 0}`)
        .then((msg) => {
              return client.doRequestAsync('GET', '/speech/info', '')
              })
        .then((msg) => {
            asrSwitch.checked = msg.body.active
        })

    if(active)
    {
        console.log('enabling audio broadcaster')
        Audio.AudioBroadcaster.active = true
    }
}


function sendCommand(cmd) {
    client.doRequestAsync('POST', `/speech/send_command?cmd=${cmd}`)
        .then((msg) => {

              })
}

function toTitleCase(phrase) {
  return phrase
    .toLowerCase()
    .split(' ')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join(' ');
};

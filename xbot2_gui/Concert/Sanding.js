function start() {

    let patch = colRepeater.itemAt(root.selectedIndex)

    var params = {
        'force': maxForceSpin.value,
        'speed': speedSpin.value,
        'xmin': patch.patchMinX,
        'xmax': patch.patchMaxX,
        'ymin': patch.patchMinY,
        'ymax': patch.patchMaxY,
        'type': patch.patchType,
        'height_level': patch.yPatchIdx,
        'robot_type': robotTypeCombo.currentText
    }

    client.doRequestAsync('POST', '/concert/sanding/start',
                          JSON.stringify(params))
    .then((response) => {
              console.log('will start sanding process')
              return client.doRequestAsync('PUT', '/process/sanding/command/start',
                                           '')
          })
    .then((response) => {
              console.log('started sanding process')
              statusText.text = 'ProcessStarted'
          })
    .catch((error) => {
               console.log('ERROR starting sanding process')
           })


}

function kill() {
    console.log('will KILL sanding process')
    return client.doRequestAsync('PUT', '/process/sanding/command/kill',
                                 '')
}

function sandingToolStartedAck() {
    client.doRequestAsync('POST', '/concert/sanding/tool_started_ack', '')
    .then((response) => {

          })
}

function construct() {
    client.doRequestAsync('GET', '/concert/sanding/configure', '')
    .then((response) => {
              console.log(JSON.stringify(response))
              robotToBreakpoints = response.breakpoints
              let robotTypes = []
              for (const [key, value] of Object.entries(robotToBreakpoints)) {
                  robotTypes.push(key)
              }
              robotTypeCombo.model = robotTypes
              robotTypeCombo.modelChanged()

          })
}

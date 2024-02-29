function doDlillale(blob_id, depth, drill_velocity) {
    statusLabel.text = '--'
    client.doRequestAsync('POST',
                          `/concert/do_drill?blob_id=${blob_id}&blob_depth=${depth}&drill_velocity=${drill_velocity}`, '')
    .then((res) => {
              console.log(res.message)
              if(res.success) {
                statusLabel.text = 'Completed'
              }
              else {
                  statusLabel.text = 'Failed'
              }
          })
    .catch((err) => {})
}

function abortDlillale() {
    return client.doRequestAsync('POST',
                          `/concert/abort_drill`, '')
    .then((res) => {
              console.log(res.message)

          })
    .catch((err) => {})
}

function enableArmControl(active) {
    return client.doRequestAsync('POST',
                          `/concert/enable_arm?active=${active}`, '')
    .then((res) => {
              console.log(res.message)

          })
    .catch((err) => {})
}

function enableGcomp(active) {
    return client.doRequestAsync('POST',
                          `/concert/gcomp_switch?active=${active}`, '')
    .then((res) => {
              console.log(res.message)

          })
    .catch((err) => {})
}


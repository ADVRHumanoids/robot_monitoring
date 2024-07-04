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
                          `/concert/enable_arm?active=${active}&task_name=${configPane.armEE}`, '')
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

function updateTaskNames() {
    client.doRequest('GET', '/cartesian/get_task_list', '',
                     function(response) {
                         var cartesianTaskNames = []
                         for(let i = 0; i < response.names.length; i++) {
                             console.log(`got task ${response.types[i]} ${response.names[i]}`)
                             if(response.types[i] === "Cartesian" ||
                                     response.types[i] === "Interaction") {
                                 cartesianTaskNames.push(response.names[i])
                             }
                         }
                         configPane.armEEOptions = cartesianTaskNames
                     })
}
